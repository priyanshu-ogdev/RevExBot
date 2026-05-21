import gymnasium as gym
import os

from .revex_loco_cfg import RevExLocoCfg

# Get the absolute path to your YAML config for the RL agent
import revex_ext
AGENT_CFG_DIR = os.path.join(os.path.dirname(revex_ext.__file__), "cfg", "train")

# Register the environment with Gymnasium
gym.register(
    id="RevEx-Loco-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": RevExLocoCfg,
        "rl_games_cfg_entry_point": f"{AGENT_CFG_DIR}/phase1_ppo.yaml",
    },
)