import argparse
import os
import torch
from omni.isaac.lab.app import AppLauncher

# 1. Parse Arguments & Launch Isaac Sim (Must happen first)
parser = argparse.ArgumentParser(description="SKRL MoE Trainer (16GB VRAM Optimized)")
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

import json

def load_frozen_experts(policy_model, device, registry_path="data/expert_registry.json"):
    """
    Surgically injects pre-trained weights from the Data Vault into the MoE.
    Dynamically syncs with the Master Factory's expert_registry.json.
    """
    print("📥 Accessing Dynamic Data Vault via Expert Registry...")
    
    # 1. Base Survival Experts (Always required for foundation)
    expert_paths = [
        "data/weights/expert_loco_v1.pth",  # Expert 0: Base Locomotion
        "data/weights/expert_agile_v1.pth"  # Expert 1: Agile Recovery
    ]
    
    # 2. Dynamic Domain Experts (Pulled from Factory Registry)
    if os.path.exists(registry_path):
        with open(registry_path, 'r') as f:
            registry = json.load(f)
            
        # Append the latest dynamically trained experts up to the MoE's capacity
        available_slots = len(policy_model.experts) - len(expert_paths)
        for expert_entry in registry.get("experts", [])[-available_slots:]:
            expert_paths.append(expert_entry["policy_path"])
    else:
        print(f"⚠️ Warning: Registry {registry_path} not found. MoE may be incomplete.")

    # 3. Weight Injection Protocol
    for i, path in enumerate(expert_paths):
        if not os.path.exists(path):
            print(f"🚨 FATAL: Missing Expert Weights at {path}")
            continue
            
        print(f"   => Injecting Expert {i}: {path}")
        checkpoint = torch.load(path, map_location=device)
        
        raw_state_dict = checkpoint.get("model", checkpoint)
        expert_state_dict = {}
        
        # Clean RL-Games formatting to match pure PyTorch nn.Sequential
        for k, v in raw_state_dict.items():
            clean_key = k.replace("a2c_network.", "").replace("actor.", "").replace("model.", "")
            
            if "value" in clean_key or "critic" in clean_key:
                continue
                
            if "mlp." in clean_key:
                clean_key = clean_key.replace("mlp.", "")
                
            expert_state_dict[clean_key] = v

        try:
            policy_model.experts[i].load_state_dict(expert_state_dict, strict=False)
            print(f"   ✅ Successfully injected Expert {i}")
        except Exception as e:
            print(f"⚠️ Warning during Expert {i} injection: {e}")
            
def main():
    set_seed(42)
    print(f"🚀 Igniting SKRL MoE Factory | Task: {args.task} | Phase: {args.phase.upper()}")
    
    # 3. Create Environment via Gymnasium Registry
    env = gym.make(args.task)
    env = SkrlVecEnvWrapper(env)
    
    # 4. Initialize Models (148-Float Tensors)
    policy_model = RevExMoEPolicy(
        observation_space=env.observation_space,
        action_space=env.action_space,
        device=env.device,
        num_experts=3,
        top_k=2,
        noise_scale=0.01 if args.phase == "router" else 0.0 # Explore only during routing
    )
    
    # Asymmetric Critic sees the privileged physics state
    critic_model = RevExCritic(
        privileged_observation_space=env.observation_space, # Mapped correctly by Wrapper
        device=env.device
    )

    # 5. Curriculum Phase Lock (The Freezing Protocol)
    if args.phase == "router":
        load_frozen_experts(policy_model, env.device)
        print("🧊 FREEZING EXPERTS: Isolating Gradients to Master Router Gate...")
        for expert in policy_model.experts:
            for param in expert.parameters():
                param.requires_grad = False
    elif args.phase == "finetune":
        print("🔥 UNFREEZING ALL: Initiating End-to-End Latent Smoothing...")
        # Note: If resuming from router, args.checkpoint should point to the trained router

    # 6. SKRL Hyperparameter Configuration (16GB VRAM Safelock)
    cfg = PPO_DEFAULT_CONFIG.copy()
    
    # Memory Geometry (Micro-batching for RTX 4070 Ti Super)
    cfg["rollouts"] = 16          
    cfg["learning_epochs"] = 8
    cfg["mini_batches"] = 4       
    cfg["mixed_precision"] = True # FP16 Tensor Core Activation (Mandatory)
    
    # PPO Mathematics
    cfg["discount_factor"] = 0.99
    cfg["lambda"] = 0.95
    cfg["grad_norm_clip"] = 1.0
    cfg["load_balancing_coef"] = 0.01 # Lambda for Aux Loss
    
    # Optimizer Injection (SGD for Router, Adam for Finetune)
    if args.phase == "router":
        cfg["optimizer"] = torch.optim.SGD
        cfg["optimizer_kwargs"] = {"lr": 1e-3, "momentum": 0.9}
        print("⚙️  Optimizer: SGD with Momentum (Gating Network Stability)")
    else:
        cfg["optimizer"] = torch.optim.Adam
        cfg["optimizer_kwargs"] = {"lr": 1e-5} # Ultra-low LR to prevent catastrophic forgetting
        print("⚙️  Optimizer: AdamW (End-to-End Smoothing)")

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
        print(f"📥 Loading Checkpoint: {args.checkpoint}")
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