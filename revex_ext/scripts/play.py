"""
Universal ASE policy player for RevExBot.
Loads a trained checkpoint and visualises the policy in real time.
"""
import argparse
import torch
import os

from omni.isaac.lab.app import AppLauncher

# 1. Parse arguments
parser = argparse.ArgumentParser(description="Play the RevEx ASE policy.")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to .pt checkpoint.")
parser.add_argument("--phase", type=int, default=2, choices=[1, 2],
                    help="Phase of the checkpoint (1 = base loco, 2 = ASE style).")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
parser.add_argument("--headless", action="store_true", default=False, help="Run without GUI.")
parser.add_argument("--skill_id", type=str, default=None,
                    help="If phase=2, force a specific skill ID (e.g., 'combat_jab').")
AppLauncher.add_app_launcher_args(parser)
args, _ = parser.parse_known_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# 2. Imports
from omni.isaac.lab_tasks.utils.wrappers.skrl import SkrlVecEnvWrapper
from revex_ext.envs.revex_ase_env import RevExAseEnv
from revex_ext.envs.revex_ase_env_cfg import RevExAseEnvCfg
from revex_ext.agents.ase_policy import ASEHistoryPolicy
from revex_ext.pipeline.motion_library_manager import MotionLibraryManager

# 3. Setup environment
env_cfg = RevExAseEnvCfg(phase=args.phase)
env_cfg.scene.num_envs = args.num_envs
env = RevExAseEnv(cfg=env_cfg, render_mode=None if args.headless else "human")
env = SkrlVecEnvWrapper(env)
device = env_cfg.sim.device

obs_dict = env.reset()
act_dim = env.action_space.shape[0]

# 🚨 FIX: style code is stored in ase_data, not a separate observation key
style_code_dim = env.unwrapped.extras["ase_data"]["z"].shape[-1]

# 4. Load policy
policy = ASEHistoryPolicy(
    obs_dim=obs_dict["policy"].shape[-1],
    critic_obs_dim=obs_dict["critic"].shape[-1],
    act_dim=act_dim,
    latent_dim=style_code_dim
).to(device)

checkpoint = torch.load(args.checkpoint, map_location=device)
policy.load_state_dict(checkpoint["policy"])
policy.eval()

# 5. Phase 2 setup
if args.phase == 2:
    lib_path = env_cfg.style_config["motion_library_path"]
    motion_manager = MotionLibraryManager(library_path=lib_path, latent_dim=style_code_dim, device=device)
    env.unwrapped.motion_library_manager = motion_manager

    if args.skill_id is not None:
        # Force skill
        for i, clip in enumerate(motion_manager.clips):
            if clip.get("skill_id") == args.skill_id:
                env.unwrapped.extras["ase_data"]["z"][:] = motion_manager.z_codes[i]
                print(f"🎯 Forcing skill: {args.skill_id}")
                break

    # Initialise _last_state for style reward computation
    env.unwrapped._last_state = obs_dict["policy"][:, -1, :].detach()

# 6. Play loop
print("\n🚀 Running policy. Press Ctrl+C to stop.")
try:
    with torch.no_grad():
        while simulation_app.is_running():
            # Read z from the environment's ase_data
            z = env.unwrapped.extras["ase_data"]["z"]
            action, _, _ = policy.get_action(obs_dict["policy"], obs_dict["critic"], z)
            
            # Step environment
            obs_dict, _, _, _, _ = env.step(action)

            # Telemetry for active skill
            if args.phase == 2:
                data = env.unwrapped.extras["ase_data"]
                skill_types = data["skill_type"]
                current_skill = skill_types[0] if isinstance(skill_types, list) else skill_types
                phase = data["phase"][0].item()
                print(f"🎭 Skill: {current_skill:15} | Phase: {phase:.2f}    ", end="\r")
except KeyboardInterrupt:
    print("\n⏹️ Stopping...")
finally:
    env.close()
    simulation_app.close()