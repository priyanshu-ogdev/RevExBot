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

def load_expert_registry(registry_path="revex_ext/data/expert_registry.json"):
    """Loads the expert registry and returns a dict of {name: path}."""
    expert_paths = {}
    
    if os.path.exists(registry_path):
        with open(registry_path, 'r') as f:
            registry = json.load(f)
        for name, path in registry.items():
            if path and os.path.exists(path):
                expert_paths[name] = path
            else:
                print(f"⚠️ Warning: Configured expert '{name}' missing valid path in registry.")
    else:
        print(f"🚨 FATAL: Registry not found at {registry_path}")
                
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
            
            # Inject directly into the policy buffers
            setattr(policy_model, f"{name}_obs_mean", rm)
            setattr(policy_model, f"{name}_obs_std", std)
            
        raw_state_dict = checkpoint.get("model", checkpoint)
        expert_state_dict = {}
        
        # Clean RL-Games formatting to match pure PyTorch nn.Sequential
        for k, v in raw_state_dict.items():
            clean_key = k.replace("a2c_network.actor.", "").replace("model.", "")
            
            if "value" in clean_key or "critic" in clean_key or "discriminator" in clean_key:
                continue
                
            # 🚨 CRITICAL FIX: Strip "mlp." to map perfectly to nn.Sequential
            if "mlp." in clean_key:
                clean_key = clean_key.replace("mlp.", "")
                
            expert_state_dict[clean_key] = v

        try:
            if name in policy_model.experts:
                policy_model.experts[name].load_state_dict(expert_state_dict, strict=True)
                print(f"   ✅ Successfully injected Expert '{name}'")
            else:
                print(f"   ⚠️ Expert '{name}' not found in policy model")
        except Exception as e:
            print(f"   ❌ FATAL Error during Expert '{name}' injection: {e}")

def main():
    set_seed(42)
    print(f"🚀 Igniting SKRL MoE Factory | Task: {args.task} | Phase: {args.phase.upper()}")
    
    # 3. Create Environment via Gymnasium Registry
    env = gym.make(args.task)
    env = SkrlVecEnvWrapper(env)
    
    # 4. Load Expert Registry & Initialize Policy
    expert_paths = load_expert_registry()
    print(f"📋 Loaded {len(expert_paths)} experts: {list(expert_paths.keys())}")
    
    # 🚨 FIXED: Pass dummy paths to bypass internal flawed loader
    dummy_paths = {k: "" for k in expert_paths.keys()}
    policy_model = RevExMoEPolicy(
        observation_space=env.observation_space,
        action_space=env.action_space,
        device=env.device,
        expert_paths=dummy_paths,  
        router_hidden_dims=[128, 64],
        top_k=2,
        noise_scale=0.01 if args.phase == "router" else 0.0,
        load_balancing_coef=0.01
    )
    
    # 🚨 FIXED: Explicitly call our surgical injector to correctly map "mlp." keys
    load_frozen_experts(policy_model, expert_paths, env.device)
    
    # 🚨 FIXED: Standard SKRL Signature for the Critic
    critic_model = RevExCritic(
        observation_space=env.observation_space,
        action_space=env.action_space,
        device=env.device
    )

    # 5. Curriculum Phase Lock (The Freezing Protocol)
    if args.phase == "router":
        print("🧊 FREEZING EXPERTS: Isolating Gradients to Master Router Gate...")
        for expert in policy_model.experts.values():
            for param in expert.parameters():
                param.requires_grad = False
    elif args.phase == "finetune":
        print("🔥 UNFREEZING ALL: Initiating End-to-End Latent Smoothing...")
        for expert in policy_model.experts.values():
            for param in expert.parameters():
                param.requires_grad = True
        for param in policy_model.gating_network.parameters():
            param.requires_grad = True

    # 6. SKRL Hyperparameter Configuration (Ada 6000 Optimized)
    cfg = PPO_DEFAULT_CONFIG.copy()
    
    cfg["rollouts"] = 64          
    cfg["learning_epochs"] = 6
    cfg["mini_batches"] = 16      
    cfg["mixed_precision"] = True 
    
    cfg["discount_factor"] = 0.99
    cfg["lambda"] = 0.95
    cfg["grad_norm_clip"] = 1.0
    
    cfg["load_balancing_coef"] = 0.01
    cfg["virtual_opp_coef"] = 0.005
    
    cfg["normalize_input"] = False
    cfg["normalize_value"] = True
    cfg["value_bootstrap"] = True
    
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

    # 8. 🚨 FIXED: Legal SKRL Trainer Assembly
    trainer = SequentialTrainer(
        env=env,
        agents=agent,
        cfg={"timesteps": 150000}
    )
    trainer.run()

    env.close()
    simulation_app.close()
    print("✅ MoE Compilation Complete.")

if __name__ == "__main__":
    main()