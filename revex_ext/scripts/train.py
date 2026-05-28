import argparse
from omni.isaac.lab.app import AppLauncher

# 1. Parse arguments (Strictly for Phase 1 & 2 Expert Forging)
parser = argparse.ArgumentParser(description="RL-Games Expert Forger for RevExBot")
parser.add_argument("--task", type=str, default="RevEx-Loco-v0", help="Registered Task name.")
parser.add_argument("--num_envs", type=int, default=None, help="Override environment count.")
parser.add_argument("--checkpoint", type=str, default=None, help="Resume from a specific .pth file.")

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# 2. Launch Isaac Sim
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# 3. Post-Launch Imports
import torch
from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab_tasks.utils.hydra import hydra_task_config
from omni.isaac.lab_tasks.utils.wrappers.rlgames import RlGamesVecEnvWrapper
from rl_games.common import env_configurations
from rl_games.torch_runner import Runner

import revex_ext 

@hydra_task_config(args.task, "rl_games_cfg")
def main(env_cfg, agent_cfg: dict):
    # Apply VRAM/Hardware overrides
    if args.num_envs is not None:
        env_cfg.scene.num_envs = args.num_envs

    print(f"🚀 Forging Expert | Task: {args.task} | Backend: RL-Games")
    
    # Initialize Environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = RlGamesVecEnvWrapper(env, device=env_cfg.sim.device)

    # Register with RL-Games Engine
    env_configurations.register("rlgpu", {
        "vecenv_type": "RLGPU",
        "env_creator": lambda **kwargs: env
    })

    # Configure Runner
    runner = Runner()
    runner.load(agent_cfg)
    
    print("🔥 Starting High-Throughput RL-Games PPO Loop...")
    
    # Execution Arguments
    run_kwargs = {"train": True, "play": False}
    if args.checkpoint:
        print(f"📥 Loading Checkpoint: {args.checkpoint}")
        run_kwargs["checkpoint"] = args.checkpoint

    # Ignite Training
    runner.run(run_kwargs)

    # Clean Exit
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()