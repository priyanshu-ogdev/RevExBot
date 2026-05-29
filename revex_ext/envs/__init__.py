"""
Gymnasium Environment Registry for RevExBot.
Consolidated root configuration for out-of-tree MoE pipeline execution.
"""
import gymnasium as gym
import os
import revex_ext

# Resolve absolute pathing for training configurations dynamically
AGENT_CFG_DIR = os.path.join(os.path.dirname(revex_ext.__file__), "cfg", "train")

# Direct class object imports for compile-time validation
from revex_ext.envs.loco.revex_loco_cfg import RevExLocoCfg
from revex_ext.envs.agile.revex_agile_cfg import RevExAgileCfg
from revex_ext.envs.skills.revex_combat_cfg import RevExCombatEnvCfg
from revex_ext.envs.skills.revex_precision_cfg import RevExPrecisionEnvCfg
from revex_ext.envs.skills.revex_dance_cfg import RevExDanceEnvCfg

# ==============================================================================
# PHASE 1 & 2: PROPRIOCEPTIVE FOUNDATION EXPERTS
# ==============================================================================

gym.register(
    id="RevEx-Loco-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": RevExLocoCfg,
        "rl_games_cfg_entry_point": os.path.join(AGENT_CFG_DIR, "phase1_loco_ppo.yaml"),
    },
)

gym.register(
    id="RevEx-Agile-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": RevExAgileCfg,
        "rl_games_cfg_entry_point": os.path.join(AGENT_CFG_DIR, "phase2_agile_ppo.yaml"),
    },
)

# ==============================================================================
# PHASE 3: ADVERSARIAL & HIGH-FIDELITY SKILL EXPERTS
# ==============================================================================

gym.register(
    id="RevEx-Combat-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": RevExCombatEnvCfg,
        "rl_games_cfg_entry_point": os.path.join(AGENT_CFG_DIR, "skill_combat_amp.yaml"),
    },
)

gym.register(
    id="RevEx-Precision-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": RevExPrecisionEnvCfg,
        "rl_games_cfg_entry_point": os.path.join(AGENT_CFG_DIR, "skill_precision_ppo.yaml"),
    },
)

gym.register(
    id="RevEx-Dance-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": RevExDanceEnvCfg,
        "rl_games_cfg_entry_point": os.path.join(AGENT_CFG_DIR, "skill_dance_amp.yaml"),
    },
)