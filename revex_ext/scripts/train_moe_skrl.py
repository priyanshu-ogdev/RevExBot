# scripts/train_moe_skrl.py
import argparse
import os
import json
import torch
from omni.isaac.lab.app import AppLauncher

# 1. Parse Arguments & Launch Isaac Sim (Must happen first)
parser = argparse.ArgumentParser(description="SKRL MoE Trainer (5-Expert RevExBot)")
parser.add_argument("--task", type=str, default="RevEx-Combat-v0", help="MoE Task to assemble.")
parser.add_argument("--phase", type=str, default="router", choices=["router", "finetune"], help="Curriculum phase.")
parser.add_argument("--checkpoint", type=str, default=None, help="Resume an MoE checkpoint.")
parser.add_argument("--expert_names", type=str, nargs="+", default=["loco", "agile", "combat", "dance", "precision"], help="Expert names for routing.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# 2. Late Imports (Post-Simulation Launch)
import gymnasium as gym
import revex_ext  # Triggers Gymnasium Env Registry
from omni.isaac.lab_tasks.utils.wrappers.skrl import SkrlVecEnvWrapper
from skrl.agents.torch.ppo import PPO_DEFAULT_CONFIG
from skrl.memories.torch import RandomMemory
from skrl.trainers.torch import SequentialTrainer
from skrl.utils import set_seed

# Custom RevEx Architecture
from revex_ext.models.moe_policy import RevExMoEPolicy, RevExCritic
from revex_ext.agents.custom_moe_ppo import MoEPPOAgent

def load_expert_registry(registry_path="data/expert_registry.json"):
    """Loads the expert registry and returns a dict of {name: path}."""
    expert_paths = {}
    
    # Base survival experts (always required)
    expert_paths["loco"] = "data/weights/expert_loco_v1.pth"
    expert_paths["agile"] = "data/weights/expert_agile_v1.pth"
    
    # Dynamic domain experts from registry
    if os.path.exists(registry_path):
        with open(registry_path, 'r') as f:
            registry = json.load(f)
        for entry in registry.get("experts", []):
            name = entry.get("style", entry.get("domain", "unknown"))
            path = entry.get("policy_path")
            if path and os.path.exists(path):
                expert_paths[name] = path
                
    return expert_paths

def load_frozen_experts(policy_model, expert_paths, device):
    """
    Surgically injects pre-trained weights from the Data Vault into the MoE.
    Handles asymmetric RL-Games checkpoint extraction.
    """
    print("📥 Accessing Dynamic Data Vault via Expert Registry...")
    
    for idx, (name, path) in enumerate(expert_paths.items()):
        if not os.path.exists(path):
            print(f"🚨 FATAL: Missing Expert Weights at {path}")
            continue
            
        print(f"   => Injecting Expert '{name}' ({idx}): {path}")
        checkpoint = torch.load(path, map_location=device)
        
        # Extract normalization stats for MoEPolicy buffers
        if "running_mean_std" in checkpoint:
            rm = checkpoint["running_mean_std"]["running_mean"].to(device)
            rv = checkpoint["running_mean_std"]["running_var"].to(device)
            std = torch.sqrt(rv) + 1e-8
            # Register as buffer in policy (handled by _build_experts if path is non-empty)
            
        raw_state_dict = checkpoint.get("model", checkpoint)
        expert_state_dict = {}
        
        # Clean RL-Games formatting to match pure PyTorch nn.Sequential
        for k, v in raw_state_dict.items():
            clean_key = k.replace("a2c_network.actor.", "").replace("model.", "")
            
            if "value" in clean_key or "critic" in clean_key or "discriminator" in clean_key:
                continue
                
            if "mlp." in clean_key:
                clean_key = clean_key.replace("mlp.", "")
                
            expert_state_dict[clean_key] = v

        try:
            if name in policy_model.experts:
                policy_model.experts[name].load_state_dict(expert_state_dict, strict=False)
                print(f"   ✅ Successfully injected Expert '{name}'")
            else:
                print(f"   ⚠️ Expert '{name}' not found in policy model")
        except Exception as e:
            print(f"⚠️ Warning during Expert '{name}' injection: {e}")

def main():
    set_seed(42)
    print(f"🚀 Igniting SKRL MoE Factory | Task: {args.task} | Phase: {args.phase.upper()}")
    
    # 3. Create Environment via Gymnasium Registry
    env = gym.make(args.task)
    env = SkrlVecEnvWrapper(env)
    
    # 4. Load Expert Registry & Initialize Policy
    expert_paths = load_expert_registry()
    print(f"📋 Loaded {len(expert_paths)} experts: {list(expert_paths.keys())}")
    
    policy_model = RevExMoEPolicy(
        observation_space=env.observation_space,
        action_space=env.action_space,
        device=env.device,
        expert_paths=expert_paths,  # Pass paths for _build_experts to handle loading
        router_hidden_dims=[128, 64],
        top_k=2,
        noise_scale=0.01 if args.phase == "router" else 0.0,
        load_balancing_coef=0.01
    )
    
    # Asymmetric Critic sees the privileged physics state
    critic_model = RevExCritic(
        privileged_observation_space=env.observation_space,
        device=env.device
    )

    # 5. Curriculum Phase Lock (The Freezing Protocol)
    if args.phase == "router":
        # Experts are already loaded and frozen by RevExMoEPolicy._build_experts
        print("🧊 FREEZING EXPERTS: Isolating Gradients to Master Router Gate...")
        # (Freezing is handled in _build_experts, but we can double-check)
        for expert in policy_model.experts.values():
            for param in expert.parameters():
                param.requires_grad = False
    elif args.phase == "finetune":
        print("🔥 UNFREEZING ALL: Initiating End-to-End Latent Smoothing...")
        for expert in policy_model.experts.values():
            for param in expert.parameters():
                param.requires_grad = True
        # Also unfreeze gating network if needed
        for param in policy_model.gating_network.parameters():
            param.requires_grad = True

    # 6. SKRL Hyperparameter Configuration (Ada 6000 Optimized)
    cfg = PPO_DEFAULT_CONFIG.copy()
    
    # 🚨 ADA SCALE: Match 16k environment training
    cfg["rollouts"] = 64          # Longer horizon for stable routing
    cfg["learning_epochs"] = 6
    cfg["mini_batches"] = 16      # More batches for 48GB VRAM
    cfg["mixed_precision"] = True # FP16 Tensor Core Activation
    
    # PPO Mathematics
    cfg["discount_factor"] = 0.99
    cfg["lambda"] = 0.95
    cfg["grad_norm_clip"] = 1.0
    
    # 🚨 MOE AUXILIARY LOSSES
    cfg["load_balancing_coef"] = 0.01
    cfg["virtual_opp_coef"] = 0.005
    
    # 🚨 CRITICAL: Disable SKRL's global normalization
    # MoEPolicy handles per-expert normalization internally
    cfg["normalize_input"] = False
    cfg["normalize_value"] = True
    cfg["value_bootstrap"] = True
    
    # Optimizer Injection (SGD for Router stability, Adam for Finetune)
    if args.phase == "router":
        cfg["optimizer"] = torch.optim.SGD
        cfg["optimizer_kwargs"] = {"lr": 1e-3, "momentum": 0.9}
        print("⚙️  Optimizer: SGD with Momentum (Gating Network Stability)")
    else:
        cfg["optimizer"] = torch.optim.Adam
        cfg["optimizer_kwargs"] = {"lr": 1e-5, "weight_decay": 0.001}
        print("⚙️  Optimizer: Adam (End-to-End Smoothing)")
        
    cfg["lr_schedule"] = "adaptive"
    cfg["kl_threshold"] = 0.012
    cfg["entropy_coef"] = 0.005
    cfg["entropy_coef_schedule"] = [(0, 0.005), (1500, 0.001)]

    # 7. Agent Assembly
    memory = RandomMemory(memory_size=cfg["rollouts"], num_envs=env.num_envs, device=env.device)
    
    agent = MoEPPOAgent(
        models={"policy": policy_model, "value": critic_model},
        memory=memory,
        cfg=cfg,
        observation_space=env.observation_space,
        action_space=env.action_space,
        device=env.device
    )
    
    if args.checkpoint:
        print(f"📥 Loading MoE Checkpoint: {args.checkpoint}")
        agent.load(args.checkpoint)

    # 8. Execution
    trainer = SequentialTrainer(cfg={"timesteps": 150000, "environment": env})
    trainer.register_agent(agent)
    trainer.run()

    env.close()
    simulation_app.close()
    print("✅ MoE Compilation Complete.")

if __name__ == "__main__":
    main()