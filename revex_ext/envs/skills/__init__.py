# envs/skills/__init__.py
import gymnasium as gym

from . import revex_scene_cfg
from . import revex_precision_cfg
from . import revex_combat_cfg
from . import revex_dance_cfg

# Register Precision (Uses native ManagerBasedRLEnv)
gym.register(
    id="RevEx-Precision-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": revex_precision_cfg.RevExPrecisionEnvCfg,
    },
)

# Register Combat (Uses native ManagerBasedRLEnv)
gym.register(
    id="RevEx-Combat-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": revex_combat_cfg.RevExCombatEnvCfg,
    },
)

# Register Dance (Uses native ManagerBasedRLEnv)
gym.register(
    id="RevEx-Dance-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": revex_dance_cfg.RevExDanceEnvCfg,
    },
)