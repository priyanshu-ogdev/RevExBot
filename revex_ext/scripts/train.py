import argparse
from omni.isaac.lab.app import AppLauncher

# 1. Parse arguments BEFORE launching Isaac Sim
parser = argparse.ArgumentParser(description="Universal Trainer for RevExBot")
parser.add_argument("--task", type=str, default="RevEx-Loco-v0", help="Name of the registered task")
parser.add_argument("--num_envs", type=int, default=None, help="Override number of environments")
# Add Isaac Lab's internal arguments (headless, physics configs, etc.)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# 2. Launch Isaac Sim App (Must happen before importing torch or envs)
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# 3. Import modules
import torch
import os
from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab_tasks.utils.hydra import hydra_task_config
from omni.isaac.lab_tasks.utils.wrappers.rlgames import RlGamesVecEnvWrapper

# Import RLGames engine
from rl_games.common import env_configurations, vecenv
from rl_games.torch_runner import Runner

# Import your extension to ensure Gymnasium registers the tasks
import revex_ext 

# 4. Define the training logic wrapped with Hydra for config management
@hydra_task_config(args.task, "rl_games_cfg")
def main(env_cfg, agent_cfg: dict):
    # Override num_envs if provided via command line
    if args.num_envs is not None:
        env_cfg.scene.num_envs = args.num_envs

    print(f"🚀 Initializing Environment for Task: {args.task}")
    
    # Create the Isaac Lab Environment
    env = ManagerBasedRLEnv(cfg=env_cfg)

    # Wrap the environment so RLGames can understand the tensor structures
    env = RlGamesVecEnvWrapper(env, device=env_cfg.sim.device)

    # Register the wrapped environment with the RLGames registry
    env_configurations.register("rlgpu", {
        "vecenv_type": "RLGPU",
        "env_creator": lambda **kwargs: env
    })

    # Configure the Runner
    runner = Runner()
    runner.load(agent_cfg)
    
    print("🔥 Starting PPO Training Loop...")
    
    # Execute the training
    runner.run({
        "train": True,
        "play": False,
    })

    # Cleanup memory and exit gracefully when training finishes (or is interrupted)
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()