from omni.isaac.lab.envs import ManagerBasedRLEnv
from .revex_loco_cfg import RevExLocoCfg

class RevExLocoEnv(ManagerBasedRLEnv):
    """
    RevEx Phase 1: Base Locomotion Environment.
    """
    def __init__(self, cfg: RevExLocoCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)