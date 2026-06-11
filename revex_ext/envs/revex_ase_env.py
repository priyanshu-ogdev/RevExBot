"""
Thin wrapper around ManagerBasedRLEnv that:
- initialises `ase_data` with safe default tensors
- reshapes the policy observation from flat (history_length * obs_dim) to
  (history_length, obs_dim) for the causal CNN.
- (optionally) attaches a MotionLibraryManager for Phase 2
"""
from omni.isaac.lab.envs import ManagerBasedRLEnv
from .revex_ase_env_cfg import RevExAseEnvCfg
from .custom_mdp import _ensure_ase_data


class RevExAseEnv(ManagerBasedRLEnv):
    def __init__(self, cfg: RevExAseEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        # Guarantee that ase_data exists before any reward or observation call
        _ensure_ase_data(self)

    def _get_observations(self) -> dict:
        """Override to reshape policy observations for the causal CNN."""
        obs_dict = super()._get_observations()
        
        # 1. Reshape Actor Policy Obs (Causal CNN expects 3D Tensor)
        if "policy" in obs_dict:
            policy_obs = obs_dict["policy"] 
            hist_len = self.cfg.observations.policy.history_length
            if hist_len > 0:
                # Isaac Lab concatenate_terms flat vector size = (hist_len) * single_obs_dim
                single_dim = policy_obs.shape[-1] // hist_len
                obs_dict["policy"] = policy_obs.view(self.num_envs, hist_len, single_dim)
        
        # 2. Critic Obs remains flat 2D Tensor (asymmetric MLP expects this)
        # obs_dict["critic"] is naturally flat because CriticCfg.history_length = 0
        
        return obs_dict