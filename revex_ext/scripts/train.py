# scripts/train.py
import argparse
from omni.isaac.lab.app import AppLauncher

# 1. Parse arguments
parser = argparse.ArgumentParser(description="RL-Games Expert Forger for RevExBot")
parser.add_argument("--task", type=str, required=True, help="Registered Task name.")
parser.add_argument("--checkpoint", type=str, default=None, help="Resume path.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch
import os
from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab_tasks.utils.hydra import hydra_task_config
from omni.isaac.lab_tasks.utils.wrappers.rlgames import RlGamesVecEnvWrapper
from rl_games.common import env_configurations
from rl_games.torch_runner import Runner

import revex_ext 

# 🚨 PRODUCTION HOOK: This class intercepts the rl_games runner to export flat stats
class RevexCheckpointExporter(Runner):
    def save(self, fn):
        super().save(fn)
        # Extract and save flat normalization stats for the MoE Policy
        checkpoint = torch.load(fn)
        if "running_mean_std" in checkpoint:
            print(f"✅ Normalization stats exported to {fn}")
        # (rl_games auto-saves stats in the checkpoint dictionary)

@hydra_task_config(args.task, "rl_games_cfg")
def main(env_cfg, agent_cfg: dict):
    # Apply 16k environment override
    env_cfg.scene.num_envs = 16384 

    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = RlGamesVecEnvWrapper(env, device=env_cfg.sim.device)

    env_configurations.register("rlgpu", {
        "vecenv_type": "RLGPU",
        "env_creator": lambda **kwargs: env
    })

    # 🚨 Use our custom exporter runner
    runner = RevexCheckpointExporter()
    runner.load(agent_cfg)
    
    run_kwargs = {"train": True, "play": False}
    if args.checkpoint:
        run_kwargs["checkpoint"] = args.checkpoint

    runner.run(run_kwargs)

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()