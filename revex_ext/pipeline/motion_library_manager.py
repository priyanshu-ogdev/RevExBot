"""
MotionLibraryManager — Centralised synchroniser for ASE style data.
Fully vectorised, O(1) step performance, safe for any library size.
"""
import json
import torch
import numpy as np

class MotionLibraryManager:
    def __init__(self, library_path: str, latent_dim: int = 16,
                 mixup_prob: float = 0.15, mixup_alpha_range: tuple = (0.2, 0.8),
                 device: str = "cuda"):
        self.device = device
        self.latent_dim = latent_dim
        self.mixup_prob = mixup_prob
        self.mixup_alpha_range = mixup_alpha_range

        with open(library_path, "r") as f:
            self.library = json.load(f)

        self.mocap_dim = self.library["mocap_dim"]
        self.clips = self.library["clips"]
        self.num_clips = len(self.clips)

        # ---- Pre‑load all tensors to GPU ----
        z_list = [clip["latent_z"] for clip in self.clips]
        self.z_codes = torch.tensor(z_list, dtype=torch.float32, device=device)

        kin_sigs = []
        for clip in self.clips:
            ks = clip.get("kinematic_signature", {})
            kin_sigs.append([
                ks.get("mean_base_vel_mag", 0.0),
                ks.get("mean_vertical_com_disp", 0.0),
                ks.get("mean_contact_duty", 0.5)
            ])
        self.kin_sigs = torch.tensor(kin_sigs, dtype=torch.float32, device=device)

        # RSI start poses
        self.start_poses = torch.tensor(
            [clip["frames"][0]["joint_pos"] for clip in self.clips],
            dtype=torch.float32, device=device
        )
        self.start_vels = torch.tensor(
            [clip["frames"][0]["joint_vel"] for clip in self.clips],
            dtype=torch.float32, device=device
        )
        self.first_frame_contacts = torch.tensor(
            [clip["frames"][0].get("contact_schedule", [0,0,0,0]) for clip in self.clips],
            dtype=torch.float32, device=device
        )

        # Per‑clip durations and total frames
        self.clip_durations = torch.tensor(
            [clip["duration_s"] for clip in self.clips],
            dtype=torch.float32, device=device
        )
        self.total_frames_per_clip = torch.tensor(
            [len(clip["frames"]) for clip in self.clips],
            dtype=torch.long, device=device
        )

        # ---- Padded timelines (O(1) per‑step) ----

        # Pre‑load joint position timelines for real transition sampling
        max_frames = self.total_frames_per_clip.max().item()
        self.padded_joint_pos = torch.zeros((self.num_clips, max_frames, self.mocap_dim), device=device)
        for i, clip in enumerate(self.clips):
            frames_count = len(clip["frames"])
            jp = torch.tensor([f["joint_pos"] for f in clip["frames"]], dtype=torch.float32, device=device)
            self.padded_joint_pos[i, :frames_count] = jp
        self.padded_contact_schedules = torch.zeros((self.num_clips, max_frames, 4), device=device)
        self.padded_stiffness = torch.ones((self.num_clips, max_frames), device=device)
        self.padded_rhythm = torch.zeros((self.num_clips, max_frames), device=device)

        type_map = {"loco": 0, "combat": 1, "dance": 2, "precision": 3}
        self.clip_skill_types = torch.zeros(self.num_clips, dtype=torch.long, device=device)

        for i, clip in enumerate(self.clips):
            frames_count = len(clip["frames"])
            cs = torch.tensor([f.get("contact_schedule", [0,0,0,0]) for f in clip["frames"]],
                              dtype=torch.float32, device=device)
            sm = torch.tensor([f.get("stiffness_mult", 1.0) for f in clip["frames"]],
                              dtype=torch.float32, device=device)
            rhy = torch.tensor([f.get("rhythm_beat", 0.0) for f in clip["frames"]],
                               dtype=torch.float32, device=device)

            self.padded_contact_schedules[i, :frames_count] = cs
            self.padded_stiffness[i, :frames_count] = sm
            self.padded_rhythm[i, :frames_count] = rhy
            self.clip_skill_types[i] = type_map.get(clip.get("skill_type", "loco"), 0)

    # ------------------------------------------------------------------
    # KNN‑based kinetic neighbourhood for mixup (safe for any library size)
    # ------------------------------------------------------------------
    def _get_safe_mixup_indices(self, primary_ids: torch.Tensor, k: int = 5) -> torch.Tensor:
        safe_k = min(k, self.num_clips - 1)
        if safe_k <= 0:
            return primary_ids   # not enough clips to mix, return primary itself
        z_primary = self.z_codes[primary_ids]
        dists = torch.cdist(z_primary, self.z_codes)
        nearest = torch.topk(dists, safe_k + 1, dim=-1, largest=False).indices[:, 1:]  # exclude self
        rand_sel = torch.randint(0, safe_k, (len(primary_ids), 1), device=self.device)
        return torch.gather(nearest, 1, rand_sel).squeeze(-1)

    # ------------------------------------------------------------------
    # SAMPLING (called by sample_ase_style event on reset)
    # ------------------------------------------------------------------
    def sample(self, env, env_ids: torch.Tensor):
        data = env.extras["ase_data"]
        num_resets = len(env_ids)

        # Ensure required tensors exist (Phase‑safe)
        if data["desired_contacts"] is None:
            data["desired_contacts"] = torch.zeros((env.num_envs, 4), device=self.device)
        if data["start_joint_pos"] is None:
            data["start_joint_pos"] = torch.zeros((env.num_envs, self.mocap_dim), device=self.device)
            data["start_joint_vel"] = torch.zeros((env.num_envs, self.mocap_dim), device=self.device)
        if not hasattr(env, "_clip_idx"):
            env._clip_idx = torch.zeros(env.num_envs, dtype=torch.long, device=self.device)
            env._clip_total_frames = torch.zeros(env.num_envs, dtype=torch.long, device=self.device)

        is_skill = torch.rand(num_resets, device=self.device) < 0.5
        is_loco = ~is_skill

        # ---- Loco episodes ----
        if is_loco.any():
            loco_ids = env_ids[is_loco]
            data["z"][loco_ids] = 0.0
            data["phase"][loco_ids] = 0.0
            data["is_skill_mode"][loco_ids] = False
            data["is_loco_mode"][loco_ids] = True
            data["is_combat_mode"][loco_ids] = False
            data["is_dance_mode"][loco_ids] = False
            data["is_precision_mode"][loco_ids] = False
            data["desired_contacts"][loco_ids] = 0.0
            data["stiffness_mult"][loco_ids] = 1.0

        # ---- Skill episodes ----
        if is_skill.any():
            skill_ids = env_ids[is_skill]
            n_skills = len(skill_ids)

            primary_idx = torch.randint(0, self.num_clips, (n_skills,), device=self.device)
            z_sampled = self.z_codes[primary_idx]

            # KNN Mixup (safe)
            do_mixup = torch.rand(n_skills, device=self.device) < self.mixup_prob
            if do_mixup.any():
                secondary_idx = self._get_safe_mixup_indices(primary_idx)
                z_secondary = self.z_codes[secondary_idx]
                alphas = torch.empty((n_skills, 1), device=self.device).uniform_(*self.mixup_alpha_range)
                z_mixed = alphas * z_sampled + (1.0 - alphas) * z_secondary
                z_sampled = torch.where(do_mixup.unsqueeze(1), z_mixed, z_sampled)

            data["z"][skill_ids] = z_sampled
            data["phase"][skill_ids] = 0.0
            data["is_skill_mode"][skill_ids] = True
            data["is_loco_mode"][skill_ids] = False

            # Vectorised skill type mapping
            sampled_types = self.clip_skill_types[primary_idx]
            data["is_combat_mode"][skill_ids] = (sampled_types == 1)
            data["is_dance_mode"][skill_ids] = (sampled_types == 2)
            data["is_precision_mode"][skill_ids] = (sampled_types == 3)

            # Frame tracking (fully vectorised)
            env._clip_idx[skill_ids] = primary_idx
            env._clip_total_frames[skill_ids] = self.total_frames_per_clip[primary_idx]

            # Vectorised state initialisation
            data["desired_contacts"][skill_ids] = self.first_frame_contacts[primary_idx]
            data["start_joint_pos"][skill_ids] = self.start_poses[primary_idx]
            data["start_joint_vel"][skill_ids] = self.start_vels[primary_idx]
            data["rhythm_array"][skill_ids, 0] = self.padded_rhythm[primary_idx, 0]

    # ------------------------------------------------------------------
    # PHASE ADVANCEMENT (Vectorised O(1) per step)
    # ------------------------------------------------------------------
    def advance_phase(self, env):
        data = env.extras["ase_data"]
        dt = env.sim.dt * env.decimation
        active_mask = data["is_skill_mode"]

        if not active_mask.any():
            return

        clip_idx = env._clip_idx[active_mask]
        durations = self.clip_durations[clip_idx]
        total_frames = env._clip_total_frames[active_mask]

        # Advance phase with per‑clip duration
        phase_inc = dt / durations
        data["phase"][active_mask] += phase_inc
        data["phase"][active_mask] %= 1.0

        current_frames = (data["phase"][active_mask] * total_frames).long()
        current_frames = torch.clamp(current_frames, max=total_frames - 1)

        # O(1) vectorised update
        data["desired_contacts"][active_mask] = self.padded_contact_schedules[clip_idx, current_frames]
        data["stiffness_mult"][active_mask] = self.padded_stiffness[clip_idx, current_frames]
        data["rhythm_array"][active_mask, 0] = self.padded_rhythm[clip_idx, current_frames]

    def sample_real_transitions(self, batch_size: int, device: torch.device) -> tuple:
        """
        Vectorised sampling of real transitions (s, s_next, z) from the library.
        Returns tensors of shape (batch_size, mocap_dim) for states and (batch_size, latent_dim) for z.
        """
        clip_indices = torch.randint(0, self.num_clips, (batch_size,), device=device)
        total_frames = self.total_frames_per_clip[clip_indices]  # (batch_size,)
        # Random frame indices, not the last frame
        max_frame = total_frames - 1
        # Clamp to avoid zero-frame clips
        max_frame = torch.clamp(max_frame, min=1)
        frame_indices = torch.randint(0, max_frame, (batch_size,), device=device)  # (batch_size,)
        next_frame_indices = frame_indices + 1

        s = self.padded_joint_pos[clip_indices, frame_indices]          # (batch_size, mocap_dim)
        s_next = self.padded_joint_pos[clip_indices, next_frame_indices]
        z = self.z_codes[clip_indices]
        return s, s_next, z