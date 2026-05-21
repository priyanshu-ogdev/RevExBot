"""
Gymnasium Environment Registry for RevExBot.
"""
import gymnasium as gym
import os
import revex_ext

# Dynamically resolve the absolute path to the YAML configurations
AGENT_CFG_DIR = os.path.join(os.path.dirname(revex_ext.__file__), "cfg", "train")

# ==============================================================================
# PHASE 1: LOCOMOTION
# ==============================================================================
gym.register(
    id="RevEx-Loco-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "revex_ext.envs.loco.revex_loco_cfg:RevExLocoCfg",
        "rl_games_cfg_entry_point": f"{AGENT_CFG_DIR}/phase1_loco_ppo.yaml",
    },
)

# ==============================================================================
# PHASE 2: AGILE LOCOMOTION & RECOVERY
# ==============================================================================
gym.register(
    id="RevEx-Agile-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "revex_ext.envs.agile.revex_agile_cfg:RevExAgileCfg",
        "rl_games_cfg_entry_point": f"{AGENT_CFG_DIR}/phase2_agile_ppo.yaml",
    },
)