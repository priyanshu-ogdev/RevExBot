"""
Initialization module for RevExBot Extension.
Registers the RL environments with Gymnasium.
"""

import gymnasium as gym
import os

# Get the absolute path to the current directory (revex_ext)
REVEX_EXT_DIR = os.path.dirname(__file__)

# ==============================================================================
# PHASE 1: LOCOMOTION
# ==============================================================================
gym.register(
    id="RevEx-Loco-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        # Using string paths prevents circular imports and speeds up CLI loading
        "env_cfg_entry_point": "revex_ext.envs.revex_loco_cfg:RevExLocoCfg",
        "rl_games_cfg_entry_point": f"{REVEX_EXT_DIR}/cfg/train/phase1_ppo.yaml",
    },
)

# ==============================================================================
# PHASE 2: AGILE LOCOMOTION & PUSH RECOVERY (Placeholder)
# ==============================================================================
# gym.register(
#     id="RevEx-Agile-v0",
#     entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
#     disable_env_checker=True,
#     kwargs={
#         "env_cfg_entry_point": "revex_ext.envs.revex_agile_cfg:RevExAgileCfg",
#         "rl_games_cfg_entry_point": f"{REVEX_EXT_DIR}/cfg/train/phase2_ppo.yaml",
#     },
# )