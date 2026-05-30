import argparse
from omni.isaac.lab.app import AppLauncher

# 1. Parse arguments BEFORE launching Isaac Sim
parser = argparse.ArgumentParser(description="Universal Visualizer for RevExBot.")
parser.add_argument("--task", type=str, default="RevEx-Loco-v0", help="Name of the task to play.")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to .pth checkpoint.")
parser.add_argument("--num_envs", type=int, default=4, help="Number of environments.")
parser.add_argument("--headless", action="store_true", default=False, help="Run headless.")

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# 2. Launch Isaac Sim App
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# 3. Import modules post-launch
import torch
from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab_tasks.utils.hydra import hydra_task_config
from omni.isaac.lab_tasks.utils.wrappers.rlgames import RlGamesVecEnvWrapper
from rl_games.torch_runner import Runner

# Register RevEx Tasks
import revex_ext

@hydra_task_config(args.task, "rl_games_cfg")
def main(env_cfg, agent_cfg: dict):
    print(f"👁️ Visualizing Task: {args.task}")
    print(f"📂 Loading Weights: {args.checkpoint}")
    
    env_cfg.scene.num_envs = args.num_envs 
    
    # Initialize Environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = RlGamesVecEnvWrapper(env, device=env_cfg.sim.device)

    # Configure Runner
    # Configure Runner
    agent_cfg["params"]["config"]["num_actors"] = env.num_envs
    
    # Force evaluation overrides to prevent minibatch size crashes
    agent_cfg["params"]["config"]["minibatch_size"] = env.num_envs * agent_cfg["params"]["config"]["horizon_length"]
    agent_cfg["params"]["config"]["mini_epochs"] = 1
    
    runner = Runner()
    runner = Runner()
    runner.load(agent_cfg)
    
    # Execute
    runner.run({
        "train": False,
        "play": True,
        "checkpoint": args.checkpoint 
    })

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()