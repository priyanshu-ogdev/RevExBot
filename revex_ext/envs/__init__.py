"""
Gymnasium Environment Registry for RevExBot.
"""
import gymnasium as gym
import os
import revex_ext

AGENT_CFG_DIR = os.path.join(os.path.dirname(revex_ext.__file__), "cfg", "train")

# --- PHASE 1 & 2: SURVIVAL ---
gym.register(
    id="RevEx-Loco-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "revex_ext.envs.loco.revex_loco_cfg:RevExLocoCfg",
        "rl_games_cfg_entry_point": f"{AGENT_CFG_DIR}/phase1_loco_ppo.yaml",
    },
)

gym.register(
    id="RevEx-Agile-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "revex_ext.envs.agile.revex_agile_cfg:RevExAgileCfg",
        "rl_games_cfg_entry_point": f"{AGENT_CFG_DIR}/phase2_agile_ppo.yaml",
    },
)

# --- PHASE 3: THE EXPERT FORGE (Used by master_factory.py) ---
gym.register(
    id="RevEx-Combat-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "revex_ext.envs.skills.revex_combat_cfg:RevExCombatEnvCfg",
        "rl_games_cfg_entry_point": f"{AGENT_CFG_DIR}/skill_combat_amp.yaml",
    },
)

gym.register(
    id="RevEx-Precision-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "revex_ext.envs.skills.revex_precision_cfg:RevExPrecisionEnvCfg",
        "rl_games_cfg_entry_point": f"{AGENT_CFG_DIR}/skill_precision_ppo.yaml",
    },
)

gym.register(
    id="RevEx-Dance-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "revex_ext.envs.skills.revex_dance_cfg:RevExDanceEnvCfg",
        "rl_games_cfg_entry_point": f"{AGENT_CFG_DIR}/skill_dance_amp.yaml",
    },
)