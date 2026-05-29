import argparse
import torch
from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser(description="SKRL 5-Expert MoE Visualizer & Telemetry")
parser.add_argument("--task", type=str, default="RevEx-Combat-v0", help="MoE Task to play.")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to Master Router .pth")
parser.add_argument("--num_envs", type=int, default=4, help="Keep low for visual clarity (1-4).")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import revex_ext
from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab_tasks.utils.parse_cfg import parse_env_cfg
from omni.isaac.lab_tasks.utils.wrappers.skrl import SkrlVecEnvWrapper
from revex_ext.models.moe_policy import RevExMoEPolicy

def main():
    print(f"👁️ Initializing MoE Visualizer for Task: {args.task}")
    print(f"📂 Loading Central Nervous System: {args.checkpoint}")
    
    env_cfg = parse_env_cfg(task_name=args.task, use_default_env_cfg=False)
    env_cfg.scene.num_envs = args.num_envs 
    
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = SkrlVecEnvWrapper(env)
    
    dummy_paths = {k: "" for k in ["loco", "agile", "combat", "dance", "precision"]}
    policy = RevExMoEPolicy(
        observation_space=env.observation_space,
        action_space=env.action_space,
        device=env.device,
        expert_paths=dummy_paths,
        top_k=2,
        noise_scale=0.0
    )
    
    checkpoint = torch.load(args.checkpoint, map_location=env.device)
    state_dict = checkpoint.get("policy", checkpoint.get("model", checkpoint))
    policy.load_state_dict(state_dict, strict=False)
    policy.eval()
    
    obs, _ = env.reset()
    print("\n🚀 Simulation Active. Watching 5-Expert Router Gates...")
    print("-" * 80)
    
    while simulation_app.is_running():
        with torch.no_grad():
            action_mean, _, extras = policy.compute({"states": obs}, role="policy")
            
            # Extract routing probabilities for Robot 0
            probs = extras["router_probs"][0].cpu().numpy()
            
            # Live 5-Expert Telemetry Dashboard
            telemetry = (
                f"🧠 ROUTER | "
                f"Loco: {probs[0]:.2f} | "
                f"Agile: {probs[1]:.2f} | "
                f"Combat: {probs[2]:.2f} | "
                f"Dance: {probs[3]:.2f} | "
                f"Precise: {probs[4]:.2f}"
            )
            print(f"{telemetry}     ", end="\r", flush=True)
            
        obs, _, _, _, _ = env.step(action_mean)

    print("\n\n⏹️ Simulation Terminated.")
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()