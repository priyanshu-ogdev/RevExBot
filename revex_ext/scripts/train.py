"""
Unified Training Script for RevExBot.
Phase 1: Base Locomotion (no style, no discriminator)
Phase 2: ASE Style Training (SGD TTUR, Dual Scalers, Teleportation Masking, Accuracy Gating)
"""
import argparse
import os
import torch
import torch.nn as nn
import numpy as np
from torch.utils.tensorboard import SummaryWriter

from omni.isaac.lab.app import AppLauncher

# ----------------------------------------------------------------------
# Argument parsing
# ----------------------------------------------------------------------
parser = argparse.ArgumentParser(description="RevEx ASE Trainer")
parser.add_argument("--phase", type=int, required=True, choices=[1, 2],
                    help="1 = Base Loco, 2 = ASE Style")
parser.add_argument("--checkpoint", type=str, default=None,
                    help="Path to a checkpoint to resume training")
parser.add_argument("--headless", action="store_true", default=True,
                    help="Run simulation without GUI")
AppLauncher.add_app_launcher_args(parser)
args, _ = parser.parse_known_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# ----------------------------------------------------------------------
# Late imports
# ----------------------------------------------------------------------
from omni.isaac.lab_tasks.utils.wrappers.skrl import SkrlVecEnvWrapper
from revex_ext.envs.revex_ase_env import RevExAseEnv
from revex_ext.envs.revex_ase_env_cfg import RevExAseEnvCfg
from revex_ext.agents.ase_policy import ASEHistoryPolicy, ASEDiscriminator
from revex_ext.pipeline.motion_library_manager import MotionLibraryManager

# ----------------------------------------------------------------------
# 1. Environment setup
# ----------------------------------------------------------------------
env_cfg = RevExAseEnvCfg(phase=args.phase)
env = RevExAseEnv(cfg=env_cfg)
env = SkrlVecEnvWrapper(env)                     # tensor‑based interface

num_envs = env_cfg.scene.num_envs                # 8192 (phase1) or 4096 (phase2)
device = env_cfg.sim.device

# Determine observation dimensions
obs_dict = env.reset()
actor_obs_seq = obs_dict["policy"]               # (num_envs, 10, single_obs_dim)
critic_obs_flat = obs_dict["critic"]             # (num_envs, critic_obs_dim)

actor_obs_dim = actor_obs_seq.shape[-1]          # single‑step observation size (~270)
critic_obs_dim = critic_obs_flat.shape[-1]
act_dim = env.action_space.shape[0]              # 39
kinematic_obs_dim = 39                           # joint positions are at indices 3:42
style_code_dim = 16

# ----------------------------------------------------------------------
# 2. Models & optimisers
# ----------------------------------------------------------------------
policy = ASEHistoryPolicy(obs_dim=actor_obs_dim, critic_obs_dim=critic_obs_dim,
                          act_dim=act_dim, latent_dim=style_code_dim).to(device)
policy.train()

if args.phase == 2:
    discriminator = ASEDiscriminator(obs_dim=kinematic_obs_dim, latent_dim=style_code_dim).to(device)
    discriminator.train()

    # Motion library manager
    lib_path = env_cfg.style_config["motion_library_path"]
    if not os.path.exists(lib_path):
        raise FileNotFoundError(f"Motion library not found: {lib_path}")
    motion_manager = MotionLibraryManager(library_path=lib_path,
                                          latent_dim=style_code_dim, device=device)
    env.unwrapped.motion_library_manager = motion_manager

    # Pass discriminator to environment for real‑time style reward
    env.unwrapped.set_discriminator(discriminator)

    # TTUR with SGD + Nesterov momentum
    policy_opt = torch.optim.SGD(policy.parameters(), lr=1e-3, momentum=0.9, weight_decay=1e-5)
    disc_opt   = torch.optim.SGD(discriminator.parameters(), lr=1e-4, momentum=0.9, weight_decay=1e-5)

    disc_scaler = torch.cuda.amp.GradScaler(enabled=torch.cuda.is_available())
else:
    policy_opt = torch.optim.SGD(policy.parameters(), lr=1e-3, momentum=0.9, weight_decay=1e-5)

policy_scaler = torch.cuda.amp.GradScaler(enabled=torch.cuda.is_available())

# ----------------------------------------------------------------------
# 3. Load checkpoint if provided
# ----------------------------------------------------------------------
start_iteration = 0
if args.checkpoint and os.path.isfile(args.checkpoint):
    print(f"📥 Loading checkpoint: {args.checkpoint}")
    ckpt = torch.load(args.checkpoint, map_location=device)
    policy.load_state_dict(ckpt["policy"])
    policy_opt.load_state_dict(ckpt["optimizer"])
    start_iteration = ckpt.get("iteration", 0) + 1
    if args.phase == 2 and "discriminator" in ckpt:
        discriminator.load_state_dict(ckpt["discriminator"])
        disc_opt.load_state_dict(ckpt["disc_optimizer"])
    print(f"   Resuming from iteration {start_iteration}")

# ----------------------------------------------------------------------
# 4. Training hyperparameters
# ----------------------------------------------------------------------
rollout_steps = 64 if args.phase == 1 else 96
total_iterations = 15000
learning_epochs = 4
mini_batch_size = 2048 if args.phase == 1 else 1024
disc_accuracy_threshold = 0.85

# Pre‑allocate rollout buffers (GPU)
actor_obs_buffer      = torch.zeros(rollout_steps, num_envs, 10, actor_obs_dim, device=device)
next_actor_obs_buffer = torch.zeros_like(actor_obs_buffer)
critic_obs_buffer     = torch.zeros(rollout_steps, num_envs, critic_obs_dim, device=device)
style_buffer          = torch.zeros(rollout_steps, num_envs, style_code_dim, device=device)
action_buffer         = torch.zeros(rollout_steps, num_envs, act_dim, device=device)
log_prob_buffer       = torch.zeros(rollout_steps, num_envs, device=device)
value_buffer          = torch.zeros(rollout_steps, num_envs, device=device)
reward_buffer         = torch.zeros(rollout_steps, num_envs, device=device)
done_buffer           = torch.zeros(rollout_steps, num_envs, device=device)

writer = SummaryWriter(log_dir=f"runs/rev_ex_ase_phase{args.phase}")

# ----------------------------------------------------------------------
# 5. Initial state for style reward (Phase 2)
# ----------------------------------------------------------------------
obs_dict = env.reset()
if args.phase == 2:
    env.unwrapped._last_state = obs_dict["policy"][:, -1, :].detach()

# ----------------------------------------------------------------------
# 6. Training loop
# ----------------------------------------------------------------------
for iteration in range(start_iteration, total_iterations):
    # ---------- Rollout ----------
    for step in range(rollout_steps):
        actor_seq   = obs_dict["policy"]            # (N, 10, dim)
        critic_flat = obs_dict["critic"]            # (N, critic_dim)
        z           = env.unwrapped.extras["ase_data"]["z"]   # (N, 16)

        with torch.no_grad():
            action, log_prob, value = policy.get_action(actor_seq, critic_flat, z)

        next_obs, reward, terminated, truncated, info = env.step(action)
        done = terminated.logical_or(truncated).float()

        # Detach all tensors
        actor_obs_buffer[step]      = actor_seq.detach()
        next_actor_obs_buffer[step] = next_obs["policy"].detach()
        critic_obs_buffer[step]     = critic_flat.detach()
        style_buffer[step]          = z.detach()
        action_buffer[step]         = action.detach()
        log_prob_buffer[step]       = log_prob.detach()
        value_buffer[step]          = value.detach()
        reward_buffer[step]         = reward.detach()
        done_buffer[step]           = done.detach()

        obs_dict = next_obs

    # ---------- GAE ----------
    with torch.no_grad():
        _, _, last_val = policy.get_action(obs_dict["policy"], obs_dict["critic"],
                                           env.unwrapped.extras["ase_data"]["z"])

    T, N = rollout_steps, num_envs
    advantages = torch.zeros_like(value_buffer)
    returns    = torch.zeros_like(value_buffer)
    gae = 0.0
    for t in reversed(range(T)):
        mask     = 1.0 - done_buffer[t]
        next_val = last_val if t == T - 1 else value_buffer[t + 1]
        delta    = reward_buffer[t] + 0.99 * next_val * mask - value_buffer[t]
        gae      = delta + 0.95 * 0.99 * mask * gae
        advantages[t] = gae
        returns[t]    = gae + value_buffer[t]

    # Flatten
    flat_actor      = actor_obs_buffer.reshape(T*N, 10, actor_obs_dim)
    flat_next_actor = next_actor_obs_buffer.reshape(T*N, 10, actor_obs_dim)
    flat_critic     = critic_obs_buffer.reshape(T*N, -1)
    flat_styles     = style_buffer.reshape(T*N, -1)
    flat_actions    = action_buffer.reshape(T*N, -1)
    flat_log_probs  = log_prob_buffer.reshape(T*N)
    flat_values     = value_buffer.reshape(T*N)
    flat_rewards    = reward_buffer.reshape(T*N)
    flat_advantages = advantages.reshape(T*N)
    flat_returns    = returns.reshape(T*N)
    flat_dones      = done_buffer.reshape(T*N)

    # ---------- Discriminator Update (Phase 2) ----------
    if args.phase == 2:
        fake_s      = flat_actor[:, -1, 3:42]        # 39‑dim joint positions
        fake_s_next = flat_next_actor[:, -1, 3:42]
        fake_z      = flat_styles

        valid_mask = (1.0 - flat_dones).bool()
        valid_fake_s      = fake_s[valid_mask]
        valid_fake_s_next = fake_s_next[valid_mask]
        valid_fake_z      = fake_z[valid_mask]

        if valid_fake_s.size(0) > 0:
            real_s, real_s_next, real_z = motion_manager.sample_real_transitions(
                batch_size=valid_fake_s.size(0), device=device
            )

            with torch.no_grad():
                with torch.cuda.amp.autocast(enabled=True):
                    eval_fake_logits = discriminator(valid_fake_s, valid_fake_s_next, valid_fake_z)
                    eval_real_logits = discriminator(real_s, real_s_next, real_z)
                real_acc = (eval_real_logits > 0).float().mean()
                fake_acc = (eval_fake_logits < 0).float().mean()
                disc_acc = (real_acc + fake_acc) / 2.0

            if disc_acc <= disc_accuracy_threshold:
                disc_opt.zero_grad()
                with torch.cuda.amp.autocast(enabled=True):
                    fake_logits = discriminator(valid_fake_s, valid_fake_s_next, valid_fake_z)
                    real_logits = discriminator(real_s, real_s_next, real_z)
                    disc_loss = 0.5 * (
                        nn.functional.binary_cross_entropy_with_logits(real_logits, torch.ones_like(real_logits)) +
                        nn.functional.binary_cross_entropy_with_logits(fake_logits, torch.zeros_like(fake_logits))
                    )
                disc_scaler.scale(disc_loss).backward()
                disc_scaler.unscale_(disc_opt)
                nn.utils.clip_grad_norm_(discriminator.parameters(), 1.0)
                disc_scaler.step(disc_opt)
                disc_scaler.update()
            else:
                disc_loss = torch.tensor(0.0, device=device)
        else:
            disc_acc = torch.tensor(0.0, device=device)
            disc_loss = torch.tensor(0.0, device=device)

    # ---------- PPO Update ----------
    indices = torch.randperm(flat_actor.size(0), device=device)
    for epoch in range(learning_epochs):
        for start in range(0, flat_actor.size(0), mini_batch_size):
            end = min(start + mini_batch_size, flat_actor.size(0))
            batch_idx = indices[start:end]

            batch_actor        = flat_actor[batch_idx]
            batch_critic       = flat_critic[batch_idx]
            batch_z            = flat_styles[batch_idx]
            batch_actions      = flat_actions[batch_idx]
            batch_old_log_probs= flat_log_probs[batch_idx]
            batch_advantages   = flat_advantages[batch_idx]
            batch_returns      = flat_returns[batch_idx]
            batch_values       = flat_values[batch_idx]

            new_log_probs, entropy, new_values = policy.evaluate_actions(
                batch_actor, batch_critic, batch_z, batch_actions
            )

            # Normalise advantages per mini‑batch
            batch_advantages = (batch_advantages - batch_advantages.mean()) / (batch_advantages.std() + 1e-8)

            ratio = torch.exp(new_log_probs - batch_old_log_probs)
            surr1 = ratio * batch_advantages
            surr2 = torch.clamp(ratio, 0.8, 1.2) * batch_advantages
            policy_loss = -torch.min(surr1, surr2).mean()

            # Value loss with clipping
            values_clipped = batch_values + torch.clamp(
                new_values - batch_values, -0.2, 0.2
            )
            v_loss_unclipped = (new_values - batch_returns).pow(2)
            v_loss_clipped   = (values_clipped - batch_returns).pow(2)
            value_loss = 0.5 * torch.max(v_loss_unclipped, v_loss_clipped).mean()

            entropy_loss = -entropy.mean()

            loss = policy_loss + 0.5 * value_loss + 0.01 * entropy_loss

            policy_opt.zero_grad()
            policy_scaler.scale(loss).backward()
            policy_scaler.unscale_(policy_opt)
            nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
            policy_scaler.step(policy_opt)
            policy_scaler.update()

    # ---------- Logging ----------
    writer.add_scalar("Loss/Policy", policy_loss.item(), iteration)
    writer.add_scalar("Loss/Value", value_loss.item(), iteration)
    if args.phase == 2:
        writer.add_scalar("Loss/Discriminator", disc_loss.item(), iteration)
        writer.add_scalar("Metrics/Disc_Accuracy", disc_acc.item(), iteration)
    writer.add_scalar("Rewards/Mean", flat_rewards.mean().item(), iteration)

    # ---------- Checkpoint every 500 iterations ----------
    if iteration % 500 == 0:
        checkpoint = {
            'policy': policy.state_dict(),
            'optimizer': policy_opt.state_dict(),
            'iteration': iteration,
        }
        if args.phase == 2:
            checkpoint.update({
                'discriminator': discriminator.state_dict(),
                'disc_optimizer': disc_opt.state_dict(),
            })
        torch.save(checkpoint, f"checkpoint_phase{args.phase}_iter{iteration}.pt")

# ----------------------------------------------------------------------
# 7. Cleanup
# ----------------------------------------------------------------------
env.close()
writer.close()
simulation_app.close()
print("Training completed.")