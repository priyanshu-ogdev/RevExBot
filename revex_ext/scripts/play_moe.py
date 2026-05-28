import argparse
import torch
from omni.isaac.lab.app import AppLauncher

# 1. Parse Arguments
parser = argparse.ArgumentParser(description="SKRL MoE Visualizer & Telemetry")
parser.add_argument("--task", type=str, default="RevEx-Combat-v0", help="MoE Task to play.")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to Master Router .pth")
parser.add_argument("--num_envs", type=int, default=4, help="Keep low for visual clarity (1-4).")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# 2. Launch Isaac Sim App
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# 3. Post-Launch Imports
import revex_ext
from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab_tasks.utils.parse_cfg import parse_env_cfg
from omni.isaac.lab_tasks.utils.wrappers.skrl import SkrlVecEnvWrapper
from revex_ext.models.moe_policy import RevExMoEPolicy

def main():
    print(f"👁️ Initializing MoE Visualizer for Task: {args.task}")
    print(f"📂 Loading Central Nervous System: {args.checkpoint}")
    
    # 4. Load & Override Environment Config dynamically
    env_cfg = parse_env_cfg(
        task_name=args.task,
        use_default_env_cfg=False
    )
    env_cfg.scene.num_envs = args.num_envs 
    
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = SkrlVecEnvWrapper(env)
    
    # 5. Initialize the MoE Policy (No Critic needed for Inference)
    policy = RevExMoEPolicy(
        observation_space=env.observation_space,
        action_space=env.action_space,
        device=env.device,
        num_experts=3,
        top_k=2,
        noise_scale=0.0 # Strict determinism for physical evaluation
    )
    
    # 6. Safely Load SKRL State Dict
    checkpoint = torch.load(args.checkpoint, map_location=env.device)
    if "policy" in checkpoint:
        policy.load_state_dict(checkpoint["policy"])
    elif "model" in checkpoint:
        policy.load_state_dict(checkpoint["model"])
    else:
        policy.load_state_dict(checkpoint)
        
    policy.eval()
    
    # 7. Simulation Loop with Live Telemetry
    obs, _ = env.reset()
    print("\n🚀 Simulation Active. Watching Router Gates...")
    print("-" * 50)
    
    while simulation_app.is_running():
        with torch.no_grad():
            # Pass observations through the MoE
            action_mean, _, extras = policy.compute({"states": obs}, role="policy")
            
            # Extract routing probabilities for Robot 0
            probs = extras["router_probs"][0].cpu().numpy()
            
            # Live Terminal Telemetry 
            # (Overwrites the same line to create a live dashboard)
            telemetry = (
                f"🧠 ROUTER | "
                f"Loco: {probs[0]:.2f} | "
                f"Agile: {probs[1]:.2f} | "
                f"Domain: {probs[2]:.2f}"
            )
            print(f"{telemetry}     ", end="\r", flush=True)
            
        # Step the environment
        obs, _, _, _, _ = env.step(action_mean)

    # Clean Exit
    print("\n\n⏹️ Simulation Terminated.")
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()