"""
Thin wrapper around ManagerBasedRLEnv that:
- initialises `ase_data` with safe default tensors
- reshapes the policy observation for the causal CNN
- injects discriminator for real‑time style reward computation
- advances motion library phase every step
"""
from omni.isaac.lab.envs import ManagerBasedRLEnv
from .revex_ase_env_cfg import RevExAseEnvCfg
from .custom_mdp import _ensure_ase_data


class RevExAseEnv(ManagerBasedRLEnv):
    def __init__(self, cfg: RevExAseEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        _ensure_ase_data(self)
        self._discriminator = None          # set by training script
        self._motion_encoder = None
        self._last_state = None             # stored before each physics step

    def set_discriminator(self, discriminator, encoder=None):
        """Enable real‑time style reward computation via the RewardManager."""
        self._discriminator = discriminator
        self._motion_encoder = encoder

    def _get_observations(self) -> dict:
        """Reshape policy observations to (num_envs, history_length, obs_dim)."""
        obs_dict = super()._get_observations()
        if "policy" in obs_dict:
            hist_len = self.cfg.observations.policy.history_length
            if hist_len > 0:
                policy_obs = obs_dict["policy"]
                single_dim = policy_obs.shape[-1] // hist_len
                obs_dict["policy"] = policy_obs.view(self.num_envs, hist_len, single_dim)
        return obs_dict

    def step(self, actions):
        # 1. Capture the state BEFORE the physics step (used by style_reward)
        if self._discriminator is not None:
            policy_obs = self.observation_manager.compute()["policy"]
            hist_len = self.cfg.observations.policy.history_length
            self._last_state = policy_obs.view(self.num_envs, hist_len, -1)[:, -1, :].detach().clone()

        # 2. Perform the physics step (RewardManager calls style_reward internally)
        returns = super().step(actions)

        # 3. Advance motion library phase (Phase 2)
        if hasattr(self, "motion_library_manager"):
            self.motion_library_manager.advance_phase(self)

        return returns