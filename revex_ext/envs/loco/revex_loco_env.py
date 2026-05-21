from omni.isaac.lab.envs import RLEnv
from .revex_loco_cfg import RevExLocoCfg

class RevExLocoEnv(RLEnv):
    """
    RevEx Phase 1 Environment.
    Logic is entirely delegated to RevExLocoCfg and executed via the Manager classes.
    """
    def __init__(self, cfg: RevExLocoCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)