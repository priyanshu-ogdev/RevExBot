from omni.isaac.lab.envs import ManagerBasedRLEnv
from .revex_agile_cfg import RevExAgileCfg

class RevExAgileEnv(ManagerBasedRLEnv):
    """
    RevEx Phase 2: Agile Locomotion Environment.
    """
    def __init__(self, cfg: RevExAgileCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)