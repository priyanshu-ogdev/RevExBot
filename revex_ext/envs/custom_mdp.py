"""
ASE-aligned Custom MDP functions for RevExBot.
Contains gating wrappers, ASE-specific observations, and custom skill rewards.
Aligned strictly with verified Isaac Lab v1.2+ API.
"""
import torch
import torch.nn.functional as F
import math
from dataclasses import dataclass

from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab.managers import SceneEntityCfg
import omni.isaac.lab.envs.mdp as mdp

# ===================================================================
# 0. DATA STRUCTURES
# ===================================================================
@dataclass
class StyleCommandCfg:
    latent_dim: int = 16
    motion_library_path: str = "data/unified_motion_library.json"
    resampling_time_range: tuple = (4.0, 6.0)
    mixup_prob: float = 0.15
    mixup_alpha_range: tuple = (0.2, 0.8)

# ===================================================================
# 1. SAFE ASE DATA ACCESS & GATING HELPERS
# ===================================================================
def _ensure_ase_data(env: ManagerBasedRLEnv) -> None:
    if "ase_data" not in env.extras:
        num, dev = env.num_envs, env.device
        env.extras["ase_data"] = {
            "z":                torch.zeros((num, 16), device=dev),
            "phase":            torch.zeros(num, device=dev),
            "skill_type":       "none",
            "is_skill_mode":    torch.zeros(num, dtype=torch.bool, device=dev),
            "is_combat_mode":   torch.zeros(num, dtype=torch.bool, device=dev),
            "is_dance_mode":    torch.zeros(num, dtype=torch.bool, device=dev),
            "is_precision_mode":torch.zeros(num, dtype=torch.bool, device=dev),
            "stiffness_mult":   torch.ones(num, device=dev),
            "disc_output":      torch.zeros(num, device=dev),
            "contact_schedule": None,
            "interaction_vectors": torch.zeros((num, 15), device=dev),
            "rhythm_array":     torch.zeros((num, 64), device=dev),
            "start_joint_pos":  None,
            "start_joint_vel":  None,
            "reference_com_vel":None,
            "desired_contacts": None,
        }

def _ase_data(env: ManagerBasedRLEnv) -> dict:
    _ensure_ase_data(env)
    return env.extras["ase_data"]

def _is_skill_active(env: ManagerBasedRLEnv) -> torch.Tensor:
    return _ase_data(env)["is_skill_mode"].float()

def _is_combat_active(env: ManagerBasedRLEnv) -> torch.Tensor:
    return _ase_data(env)["is_combat_mode"].float()

def _is_dance_active(env: ManagerBasedRLEnv) -> torch.Tensor:
    return _ase_data(env)["is_dance_mode"].float()

def _is_precision_active(env: ManagerBasedRLEnv) -> torch.Tensor:
    return _ase_data(env)["is_precision_mode"].float()

# ===================================================================
# 2. GATED WRAPPERS (Dynamically Modulated Rewards)
# ===================================================================
def tracking_lin_vel_gated(env: ManagerBasedRLEnv, *args, **kwargs) -> torch.Tensor:
    raw = mdp.track_lin_vel_xy_exp(env, *args, **kwargs)
    return raw * (1.0 - _is_skill_active(env))

def tracking_ang_vel_gated(env: ManagerBasedRLEnv, *args, **kwargs) -> torch.Tensor:
    raw = mdp.track_ang_vel_z_exp(env, *args, **kwargs)
    return raw * (1.0 - _is_skill_active(env))

def heading_alignment_gated(env: ManagerBasedRLEnv, *args, **kwargs) -> torch.Tensor:
    raw = mdp.track_heading_exp(env, *args, **kwargs)
    return raw * (1.0 - _is_skill_active(env))

def torso_upright_gated(env: ManagerBasedRLEnv, *args, **kwargs) -> torch.Tensor:
    raw = mdp.body_projected_gravity_l2(env, *args, **kwargs)
    return raw * (1.0 - _is_skill_active(env))

def tactical_engagement_gated(env: ManagerBasedRLEnv, *args, **kwargs) -> torch.Tensor:
    raw = mdp.target_position_l2(env, *args, **kwargs)
    return raw * _is_combat_active(env)

def rhythm_synchronization_reward_gated(env: ManagerBasedRLEnv) -> torch.Tensor:
    return rhythm_synchronization_reward(env) * _is_dance_active(env)

def formation_harmony_reward_gated(env: ManagerBasedRLEnv) -> torch.Tensor:
    return formation_harmony_reward(env) * _is_dance_active(env)

def angular_fluidity_penalty_gated(env: ManagerBasedRLEnv) -> torch.Tensor:
    return angular_fluidity_penalty(env) * _is_dance_active(env)

def palm_alignment_reward_gated(env: ManagerBasedRLEnv) -> torch.Tensor:
    return palm_alignment_reward(env) * _is_precision_active(env)

def soft_grasp_impedance_reward_gated(env: ManagerBasedRLEnv) -> torch.Tensor:
    return soft_grasp_impedance_reward(env) * _is_precision_active(env)

def relative_velocity_sync_gated(env: ManagerBasedRLEnv, *args, **kwargs) -> torch.Tensor:
    raw = mdp.object_vel_rel(env, *args, **kwargs)
    return raw * _is_precision_active(env)

# ===================================================================
# 3. OBSERVATIONS
# ===================================================================
def get_current_style_code(env: ManagerBasedRLEnv) -> torch.Tensor:
    return _ase_data(env)["z"]

def get_encoded_phase(env: ManagerBasedRLEnv) -> torch.Tensor:
    phi = _ase_data(env)["phase"]
    sin_part = torch.sin(2.0 * math.pi * phi).unsqueeze(-1)
    cos_part = torch.cos(2.0 * math.pi * phi).unsqueeze(-1)
    return torch.cat([sin_part, cos_part], dim=-1)

def end_effector_positions(env: ManagerBasedRLEnv) -> torch.Tensor:
    robot = env.scene["robot"]
    # Safely extract integer indices from find_bodies tuple return
    pelvis_idx = robot.find_bodies("pelvis_link")[0][0]
    lh = robot.find_bodies("lh_hand_palm")[0][0]
    rh = robot.find_bodies("rh_hand_palm")[0][0]
    lf = robot.find_bodies("left_toe_link")[0][0]
    rf = robot.find_bodies("right_toe_link")[0][0]

    pelvis_pos = robot.data.body_pos_w[:, pelvis_idx, :]
    pos = lambda idx: robot.data.body_pos_w[:, idx, :] - pelvis_pos
    return torch.cat([pos(lh), pos(rh), pos(lf), pos(rf)], dim=-1)

def get_interaction_vectors(env: ManagerBasedRLEnv, k: int = 5, dropout_prob: float = 0.05) -> torch.Tensor:
    output = _ase_data(env)["interaction_vectors"]
    if env.training:
        mask = (torch.rand(env.num_envs, 1, device=env.device) > dropout_prob).float()
        output = output * mask
    return output

def get_auxiliary_sensor_array(env: ManagerBasedRLEnv, dropout_prob: float = 0.05) -> torch.Tensor:
    data = _ase_data(env)
    output = torch.zeros((env.num_envs, 64), device=env.device)
    if data["is_combat_mode"].any() and "spatial_awareness_raycaster" in env.scene.sensors:
        combat_mask = data["is_combat_mode"].float().unsqueeze(-1)
        distances = env.scene.sensors["spatial_awareness_raycaster"].data.distance_to_hits
        output = distances * combat_mask
    if data["is_dance_mode"].any():
        dance_mask = data["is_dance_mode"].float().unsqueeze(-1)
        output = output + data["rhythm_array"] * dance_mask
    if env.training:
        mask = (torch.rand(env.num_envs, 1, device=env.device) > dropout_prob).float()
        output = output * mask
    return output

# ===================================================================
# 4. STYLE REWARD & CONTACT SCHEDULE (Self-Gating)
# ===================================================================
def style_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Computes real-time ASE discriminator style reward. Gated to Phase 2 skills."""
    if not hasattr(env.unwrapped, "_discriminator") or env.unwrapped._discriminator is None:
        return torch.zeros(env.num_envs, device=env.device)

    s = env.unwrapped._last_state          # full ~270-dim observation
    
    # Fetch fresh observation after step
    policy_obs = env.observation_manager.compute_group("policy")
    hist_len = env.cfg.observations.policy.history_length
    s_next = policy_obs.view(env.num_envs, hist_len, -1)[:, -1, :]

    # 🚨 CRITICAL: slice to 39-dim joint positions (indices 3:42) to match discriminator's obs_dim
    s_kin = s[:, 3:42]
    s_next_kin = s_next[:, 3:42]
    
    z = _ase_data(env)["z"]

    with torch.no_grad():
        with torch.cuda.amp.autocast(enabled=True):
            disc_logits = env.unwrapped._discriminator(s_kin, s_next_kin, z).squeeze(-1)

    p_real = torch.sigmoid(disc_logits)
    raw_reward = torch.exp(-2.0 * torch.clamp(1.0 - p_real, min=0.0))
    return raw_reward * _is_skill_active(env)

def track_contact_schedule(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg, threshold: float = 1.0) -> torch.Tensor:
    data = _ase_data(env)
    desired = data["desired_contacts"]
    if desired is None:
        return torch.zeros(env.num_envs, device=env.device)
    
    sensor = env.scene.sensors[sensor_cfg.name]
    contact_forces = torch.norm(sensor.data.net_forces_w, dim=-1)
    actual = (contact_forces > threshold).float()
    
    min_bodies = min(actual.shape[1], desired.shape[1])
    actual = actual[:, :min_bodies]
    desired = desired[:, :min_bodies]
    
    raw_reward = -torch.mean(torch.square(actual - desired), dim=-1)
    return raw_reward * _is_skill_active(env)

# ===================================================================
# 5. CORE SKILL & PHYSICS REWARDS
# ===================================================================
def contact_strike_reward(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg, target_cfg: SceneEntityCfg) -> torch.Tensor:
    sensor = env.scene.sensors[sensor_cfg.name]
    robot = env.scene["robot"]
    
    force_mag = torch.norm(sensor.data.net_forces_w, dim=-1).sum(dim=1)
    palm_idx = robot.find_bodies("rh_hand_palm")[0][0]
    wrist_lin_vel = robot.data.body_vel_w[:, palm_idx, :3]
    wrist_speed = torch.norm(wrist_lin_vel, dim=-1)
    
    contact_mask = (force_mag > 5.0).float()
    return contact_mask * wrist_speed * 0.1

def reference_com_velocity_tracking(env: ManagerBasedRLEnv) -> torch.Tensor:
    ref_vel = _ase_data(env).get("reference_com_vel")
    if ref_vel is None:
        return torch.zeros(env.num_envs, device=env.device)
    actual_vel = env.scene["robot"].data.root_lin_vel_w
    error = torch.norm(actual_vel - ref_vel, dim=-1)
    return -error

def rhythm_synchronization_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    beat = _ase_data(env)["rhythm_array"][:, 0]
    joint_energy = torch.norm(env.scene["robot"].data.joint_vel, dim=-1)
    alignment = joint_energy * torch.clamp(beat, min=0.0)
    return torch.clamp(alignment, max=1.5)

def formation_harmony_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    anchors = _ase_data(env)["interaction_vectors"][:, :15].view(-1, 5, 3)
    dists = torch.norm(anchors, dim=-1)
    return -torch.exp(-torch.square(dists - 1.3) / 0.2).mean(dim=-1)

def angular_fluidity_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    curr_vel = env.scene["robot"].data.root_ang_vel_w
    if not hasattr(env, "_prev_ang_vel"):
        env._prev_ang_vel = torch.zeros_like(curr_vel)
    
    env._prev_ang_vel[env.reset_buf] = 0.0 # Clear buffer on reset
    
    ang_acc = torch.norm(curr_vel - env._prev_ang_vel, dim=-1)
    env._prev_ang_vel = curr_vel.detach().clone()
    return ang_acc

def palm_alignment_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    if "target_object" not in env.scene.rigid_objects:
        return torch.zeros(env.num_envs, device=env.device)
    robot = env.scene["robot"]
    target = env.scene.rigid_objects["target_object"]
    palm_idx = robot.find_bodies("rh_hand_palm")[0][0]
    palm_pos = robot.data.body_pos_w[:, palm_idx, :]
    vec_to_target = F.normalize(target.data.root_pos_w - palm_pos + 1e-6, dim=-1)

    palm_quat = robot.data.body_quat_w[:, palm_idx, :]
    q_w, q_vec = palm_quat[:, 0:1], palm_quat[:, 1:4]
    v = torch.tensor([0.0, 0.0, 1.0], device=env.device).expand(env.num_envs, -1)
    t = 2.0 * torch.cross(q_vec, v, dim=-1)
    palm_normal = F.normalize(v + q_w * t + torch.cross(q_vec, t, dim=-1) + 1e-6, dim=-1)

    return torch.clamp(torch.sum(vec_to_target * palm_normal, dim=-1) - 0.5, min=0.0)

def soft_grasp_impedance_reward(env: ManagerBasedRLEnv, slip_margin: float = 1.2) -> torch.Tensor:
    if "wrist_contact_sensor" not in env.scene.sensors:
        return torch.zeros(env.num_envs, device=env.device)
    sensor = env.scene.sensors["wrist_contact_sensor"]
    target = env.scene.rigid_objects["target_object"]
    
    total_force = torch.norm(sensor.data.net_forces_w, dim=-1).sum(dim=1)
    obj_mass = getattr(target.data, 'root_mass', torch.ones(env.num_envs, 1, device=env.device))
    
    ideal_grip = obj_mass.squeeze(-1) * 9.81 * slip_margin
    return -torch.clamp(torch.abs(total_force - ideal_grip), max=1.0)

def angular_momentum_conservation_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    robot = env.scene[asset_cfg.name]
    ang_mom_z = robot.data.root_ang_vel_w[:, 2] * robot.data.root_mass_w.squeeze(-1)

    if not hasattr(env, "_prev_ang_mom_z"):
        env._prev_ang_mom_z = torch.zeros_like(ang_mom_z)
    
    env._prev_ang_mom_z[env.reset_buf] = 0.0 # Clear buffer on reset
    
    delta_ang_mom = torch.abs(ang_mom_z - env._prev_ang_mom_z)
    env._prev_ang_mom_z = ang_mom_z.detach().clone()

    return -torch.clamp(delta_ang_mom, max=10.0) * 0.1

def com_projection_stability_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    robot = env.scene[asset_cfg.name]
    com_pos = robot.data.root_pos_w[:, :2]

    lf_idx = robot.find_bodies("left_toe_link")[0][0]
    rf_idx = robot.find_bodies("right_toe_link")[0][0]
    lf_pos = robot.data.body_pos_w[:, lf_idx, :2]
    rf_pos = robot.data.body_pos_w[:, rf_idx, :2]

    sensor = env.scene.sensors[sensor_cfg.name]
    contact_forces = torch.norm(sensor.data.net_forces_w, dim=-1)
    lf_contact = (contact_forces[:, 0] > 1.0).float()
    rf_contact = (contact_forces[:, 1] > 1.0).float()

    support_centre = (lf_pos * lf_contact.unsqueeze(-1) + rf_pos * rf_contact.unsqueeze(-1)) / \
                     (lf_contact + rf_contact + 1e-6).unsqueeze(-1)

    dist = torch.norm(com_pos - support_centre, dim=-1)
    planted = ((lf_contact + rf_contact) > 0.5).float()
    return -dist * planted

def power_consumption(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    asset = env.scene[asset_cfg.name]
    if asset_cfg.joint_ids is not None:
        torque, vel = asset.data.applied_torque[:, asset_cfg.joint_ids], asset.data.joint_vel[:, asset_cfg.joint_ids]
    else:
        torque, vel = asset.data.applied_torque, asset.data.joint_vel
    return torch.sum(torch.clamp(torch.abs(torque) * torch.abs(vel), max=5000.0), dim=1)

def arm_swing_symmetry(env: ManagerBasedRLEnv, left_arm_cfg: SceneEntityCfg, right_arm_cfg: SceneEntityCfg,
                       left_leg_cfg: SceneEntityCfg, right_leg_cfg: SceneEntityCfg) -> torch.Tensor:
    def get_vel(cfg): return env.scene[cfg.name].data.joint_vel[:, cfg.joint_ids].mean(dim=1)
    l_arm_dir = torch.tanh(get_vel(left_arm_cfg))
    r_arm_dir = torch.tanh(get_vel(right_arm_cfg))
    l_leg_dir = torch.tanh(get_vel(left_leg_cfg))
    r_leg_dir = torch.tanh(get_vel(right_leg_cfg))
    return -(torch.square(l_arm_dir - r_leg_dir) + torch.square(r_arm_dir - l_leg_dir))

# ===================================================================
# 6. EVENTS (RSI, Spawning, Sampling)
# ===================================================================
def reset_to_reference_pose(env: ManagerBasedRLEnv, env_ids: torch.Tensor) -> None:
    data = _ase_data(env)
    ref_pose, ref_vel = data.get("start_joint_pos"), data.get("start_joint_vel")
    if ref_pose is not None and ref_vel is not None:
        if ref_pose.dim() == 2 and ref_pose.shape[0] == 1:
            ref_pose, ref_vel = ref_pose.expand(len(env_ids), -1), ref_vel.expand(len(env_ids), -1)
        else:
            ref_pose, ref_vel = ref_pose[env_ids], ref_vel[env_ids]
        env.scene["robot"].write_joint_state_to_sim(ref_pose, ref_vel, env_ids=env_ids)

def spawn_target_object(env: ManagerBasedRLEnv, env_ids: torch.Tensor) -> None:
    n = len(env_ids)
    pos = torch.zeros((n, 3), device=env.device)
    pos[:, 0] = 0.3 + 0.8 * torch.rand(n, device=env.device)
    pos[:, 1] = (torch.rand(n, device=env.device) - 0.5) * 1.0
    pos[:, 2] = 0.5 + 0.6 * torch.rand(n, device=env.device)
    env.scene.rigid_objects["target_object"].write_root_pose_to_sim(pos, env_ids=env_ids)

def apply_weapon_physics(env: ManagerBasedRLEnv, env_ids: torch.Tensor) -> None:
    robot = env.scene["robot"]
    palm_idx = robot.find_bodies("rh_hand_palm")[0][0]
    env.scene.rigid_objects["weapon_prop"].write_root_pose_to_sim(robot.data.body_pos_w[env_ids, palm_idx, :], env_ids=env_ids)

def sample_ase_style(env: ManagerBasedRLEnv, env_ids: torch.Tensor) -> None:
    if hasattr(env, "motion_library_manager"):
        env.motion_library_manager.sample(env, env_ids)

def discriminator_task_space_bias(state: torch.Tensor, scale_factors: torch.Tensor) -> torch.Tensor:
    return state * scale_factors.unsqueeze(0)