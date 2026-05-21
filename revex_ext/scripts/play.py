import argparse
from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play a trained RevExBot policy.")
parser.add_argument("--task", type=str, default="RevEx-Loco-v0", help="Name of the task.")
# Notice: headless is FALSE by default here!
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch
from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab_tasks.utils.hydra import hydra_task_config
from omni.isaac.lab_tasks.utils.wrappers.rlgames import RlGamesVecEnvWrapper
from rl_games.torch_runner import Runner
import revex_ext

@hydra_task_config(args.task, "rl_games_cfg")
def main(env_cfg, agent_cfg: dict):
    # Spawn only a few environments for visual inspection
    env_cfg.scene.num_envs = 4 
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = RlGamesVecEnvWrapper(env, device=env_cfg.sim.device)

    # Configure Runner for PLAY mode (no learning)
    agent_cfg["train"]["params"]["config"]["num_actors"] = env.num_envs
    runner = Runner()
    runner.load(agent_cfg)
    
    # Load the best checkpoint generated during training
    # (Ensure you point this to the actual saved .pth file in your runs directory)
    runner.run({
        "train": False,
        "play": True,
        "checkpoint": "runs/revex_loco_phase1/nn/revex_loco_phase1.pth" 
    })

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()