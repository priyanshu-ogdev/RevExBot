import torch
import pytest
from revex_ext.envs.revex_ase_env import RevExAseEnv
from revex_ext.envs.revex_ase_env_cfg import RevExAseEnvCfg

def test_out_of_bounds_no_nan(isaac_sim):
    """Feeding massive actions should not produce NaN observations."""
    cfg = RevExAseEnvCfg(phase=1)
    cfg.scene.num_envs = 1
    
    env = RevExAseEnv(cfg=cfg)
    env.reset()
    
    # Send a deliberately extreme action to test policy robustness
    action = torch.full((1, 39), 100.0, device=env.device)
    obs, _, _, _, _ = env.step(action)
    
    assert not torch.isnan(obs["policy"]).any(), "Environment output contains NaNs under extreme inputs!"
    env.close()