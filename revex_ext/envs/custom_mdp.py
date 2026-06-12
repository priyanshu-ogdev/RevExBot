"""
ASE-aligned Custom MDP functions for RevExBot.
Contains ONLY genuinely custom logic that has no stable mdp equivalent.
All standard observations, rewards, and events are delegated to omni.isaac.lab.envs.mdp.
All domain-specific reward functions are fully vectorised — the dynamic modulators in
revex_ase_env_cfg.py handle gating, so no string‑based skill_type checks remain.
"""
import torch
import torch.nn.functional as F
import math
from dataclasses import dataclass
from typing import Optional

from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab.managers import SceneEntityCfg

# ===================================================================
# 0. DATA STRUCTURES
# ===================================================================
@dataclass
class StyleCommandCfg:
    """Configuration for style-command sampling (used by MotionLibraryManager)."""
    latent_dim: int = 16
    motion_library_path: str = "data/unified_motion_library.json"
    resampling_time_range: tuple = (4.0, 6.0)
    mixup_prob: float = 0.15
    mixup_alpha_range: tuple = (0.2, 0.8)

# ===================================================================
# 1. SAFE ASE DATA ACCESS
# ===================================================================
def _ensure_ase_data(env: ManagerBasedRLEnv) -> None:
    """One-time initialization of ase_data with safe default tensors."""
    if "ase_data" not in env.extras:
        num = env.num_envs
        dev = env.device
        env.extras["ase_data"] = {
            "z":                torch.zeros((num, 16), device=dev),
            "phase":            torch.zeros(num, device=dev),
            "skill_type":       "none",
            "is_skill_mode":    torch.zeros(num, dtype=torch.bool, device=dev),
            "is_loco_mode":     torch.ones(num, dtype=torch.bool, device=dev),
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
    """Guaranteed to return a fully populated dict after first call."""
    _ensure_ase_data(env)
    return env.extras["ase_data"]

# ===================================================================
# 2. STYLE-CONDITIONING OBSERVATIONS
# ===================================================================
def get_current_style_code(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Return the current latent style code z (16-dim, zeros in Phase 1)."""
    return _ase_data(env)["z"]

def get_encoded_phase(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Trigonometric encoding of skill phase φ ∈ [0,1] → [sin(2πφ), cos(2πφ)]."""
    phi = _ase_data(env)["phase"]
    sin_part = torch.sin(2.0 * math.pi * phi).unsqueeze(-1)
    cos_part = torch.cos(2.0 * math.pi * phi).unsqueeze(-1)
    return torch.cat([sin_part, cos_part], dim=-1)

# ===================================================================
# 3. CORE CUSTOM OBSERVATIONS
# ===================================================================
def end_effector_positions(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Cartesian positions (12-dim) of hands and feet relative to pelvis."""
    robot = env.scene["robot"]
    pelvis_idx = robot.find_bodies("pelvis_link")[0]
    lh = robot.find_bodies("lh_hand_palm")[0]
    rh = robot.find_bodies("rh_hand_palm")[0]
    lf = robot.find_bodies("left_toe_link")[0]
    rf = robot.find_bodies("right_toe_link")[0]

    pelvis_pos = robot.data.body_pos_w[:, pelvis_idx, :]
    pos = lambda idx: robot.data.body_pos_w[:, idx, :] - pelvis_pos
    return torch.cat([pos(lh), pos(rh), pos(lf), pos(rf)], dim=-1)

def get_interaction_vectors(env: ManagerBasedRLEnv, k: int = 5, dropout_prob: float = 0.05) -> torch.Tensor:
    """15-dim threat/formation vectors with structured dropout."""
    output = _ase_data(env)["interaction_vectors"]
    if env.training:
        mask = (torch.rand(env.num_envs, 1, device=env.device) > dropout_prob).float()
        output = output * mask
    return output

def get_auxiliary_sensor_array(env: ManagerBasedRLEnv, dropout_prob: float = 0.05) -> torch.Tensor:
    """64-dim spatial lidar (combat) or rhythm array (dance) with structured dropout."""
    data = _ase_data(env)
    output = torch.zeros((env.num_envs, 64), device=env.device)
    if data["is_combat_mode"].any() and "spatial_awareness_raycaster" in env.scene.sensors:
        # vectorised: only compute real values for combat envs
        combat_mask = data["is_combat_mode"].float().unsqueeze(-1)
        output = env.scene.sensors["spatial_awareness_raycaster"].data.ray_hits_w * combat_mask
    if data["is_dance_mode"].any():
        dance_mask = data["is_dance_mode"].float().unsqueeze(-1)
        output = output + data["rhythm_array"] * dance_mask
    if env.training:
        mask = (torch.rand(env.num_envs, 1, device=env.device) > dropout_prob).float()
        output = output * mask
    return output

# ===================================================================
# 4. STYLE REWARD & CONTACT SCHEDULE
# ===================================================================
def style_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Computes the ASE style reward using the real‑time discriminator.
    Uses an exponential positive bound [0, 1] to avoid suicide policies.
    """
    if not hasattr(env.unwrapped, "_discriminator") or env.unwrapped._discriminator is None:
        return torch.zeros(env.num_envs, device=env.device)

    s = env.unwrapped._last_state

    # The environment has already stepped, so we fetch the fresh observation
    policy_obs = env.observation_manager.compute()["policy"]
    hist_len = env.cfg.observations.policy.history_length
    s_next = policy_obs.view(env.num_envs, hist_len, -1)[:, -1, :]

    z = _ase_data(env)["z"]

    with torch.no_grad():
        with torch.cuda.amp.autocast(enabled=True):
            disc_logits = env.unwrapped._discriminator(s, s_next, z).squeeze(-1)

    p_real = torch.sigmoid(disc_logits)
    # Strictly positive reward, bounded in [exp(-2), 1]
    return torch.exp(-2.0 * torch.clamp(1.0 - p_real, min=0.0))

def track_contact_schedule(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg, threshold: float = 1.0) -> torch.Tensor:
    """Penalises mismatch between desired contacts (from mocap) and actual foot contact states."""
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
    return -torch.mean(torch.square(actual - desired), dim=-1)

# ===================================================================
# 5. COMBAT REWARDS
# ===================================================================
def contact_strike_reward(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg, target_cfg: SceneEntityCfg) -> torch.Tensor:
    """Rewards high-velocity wrist contact with the target object."""
    sensor = env.scene.sensors[sensor_cfg.name]
    robot = env.scene["robot"]
    
    force_mag = torch.norm(sensor.data.net_forces_w, dim=-1).sum(dim=1)
    palm_idx = robot.find_bodies("rh_hand_palm")[0]
    wrist_lin_vel = robot.data.body_vel_w[:, palm_idx, :3]
    wrist_speed = torch.norm(wrist_lin_vel, dim=-1)
    
    contact_mask = (force_mag > 5.0).float()
    return contact_mask * wrist_speed * 0.1

def reference_com_velocity_tracking(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Auxiliary reward for matching the reference clip's CoM velocity profile."""
    ref_vel = _ase_data(env).get("reference_com_vel")
    if ref_vel is None:
        return torch.zeros(env.num_envs, device=env.device)
    robot = env.scene["robot"]
    actual_vel = robot.data.root_lin_vel_w
    error = torch.norm(actual_vel - ref_vel, dim=-1)
    return -error

# ===================================================================
# 6. DANCE REWARDS (Fully Vectorised — Modulators handle gating)
# ===================================================================
def rhythm_synchronization_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Rewards peaks in joint velocity energy aligning with the musical beat."""
    data = _ase_data(env)
    beat = data["rhythm_array"][:, 0]
    joint_energy = torch.norm(env.scene["robot"].data.joint_vel, dim=-1)
    alignment = joint_energy * torch.clamp(beat, min=0.0)
    return torch.clamp(alignment, max=1.5)

def formation_harmony_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalises deviation from virtual formation offsets."""
    data = _ase_data(env)
    anchors = data["interaction_vectors"][:, :15].view(-1, 5, 3)
    dists = torch.norm(anchors, dim=-1)
    return -torch.exp(-torch.square(dists - 1.3) / 0.2).mean(dim=-1)

def angular_fluidity_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalises high-frequency jerky rotational spins."""
    if not hasattr(env, "_prev_ang_vel"):
        env._prev_ang_vel = torch.zeros((env.num_envs, 3), device=env.device)
    curr_vel = env.scene["robot"].data.root_ang_vel_w
    ang_acc = torch.norm(curr_vel - env._prev_ang_vel, dim=-1)
    env._prev_ang_vel = curr_vel.detach().clone()
    return ang_acc

# ===================================================================
# 7. PRECISION REWARDS (Fully Vectorised — Modulators handle gating)
# ===================================================================
def palm_alignment_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Rewards aligning the palm's outward normal with the vector to the target."""
    data = _ase_data(env)
    if "target_object" not in env.scene.rigid_objects:
        return torch.zeros(env.num_envs, device=env.device)
    robot = env.scene["robot"]
    target = env.scene.rigid_objects["target_object"]
    palm_idx = robot.find_bodies("rh_hand_palm")[0]
    palm_pos = robot.data.body_pos_w[:, palm_idx, :]
    target_pos = target.data.root_pos_w
    vec_to_target = F.normalize(target_pos - palm_pos + 1e-6, dim=-1)

    palm_quat = robot.data.body_quat_w[:, palm_idx, :]
    q_w, q_vec = palm_quat[:, 0:1], palm_quat[:, 1:4]
    v = torch.tensor([0.0, 0.0, 1.0], device=env.device).expand(env.num_envs, -1)
    t = 2.0 * torch.cross(q_vec, v, dim=-1)
    palm_normal = v + q_w * t + torch.cross(q_vec, t, dim=-1)
    palm_normal = F.normalize(palm_normal + 1e-6, dim=-1)

    cos_sim = torch.sum(vec_to_target * palm_normal, dim=-1)
    return torch.clamp(cos_sim - 0.5, min=0.0)

def soft_grasp_impedance_reward(env: ManagerBasedRLEnv, slip_margin: float = 1.2) -> torch.Tensor:
    """Penalises over-/under-gripping by comparing wrist force to ideal grip force."""
    data = _ase_data(env)
    if "wrist_contact_sensor" not in env.scene.sensors:
        return torch.zeros(env.num_envs, device=env.device)
    sensor = env.scene.sensors["wrist_contact_sensor"]
    target = env.scene.rigid_objects["target_object"]
    
    total_force = torch.norm(sensor.data.net_forces_w, dim=-1).sum(dim=1)
    obj_mass = getattr(target.data, 'root_mass', getattr(target.data, 'mass', torch.ones(env.num_envs, 1, device=env.device)))
    object_mass = obj_mass.squeeze(-1)
    
    ideal_grip = object_mass * 9.81 * slip_margin
    error = torch.abs(total_force - ideal_grip)
    return -torch.clamp(error, max=1.0)

# ===================================================================
# 8. PHYSICS-PRINCIPLE REWARDS (zero-shot generalization)
# ===================================================================
def angular_momentum_conservation_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalises changes in total angular momentum around the vertical axis."""
    robot = env.scene[asset_cfg.name]
    ang_mom_z = robot.data.root_ang_vel_w[:, 2] * robot.data.root_inertia_w[:, 2, 2]

    if not hasattr(env, "_prev_ang_mom_z"):
        env._prev_ang_mom_z = ang_mom_z.clone()
    delta_ang_mom = torch.abs(ang_mom_z - env._prev_ang_mom_z)
    env._prev_ang_mom_z = ang_mom_z.detach().clone()

    return -torch.clamp(delta_ang_mom, max=10.0) * 0.1

def com_projection_stability_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Rewards keeping the projected CoM within the support polygon of the stance foot/feet."""
    robot = env.scene[asset_cfg.name]
    com_pos = robot.data.root_pos_w[:, :2]

    lf_idx = robot.find_bodies("left_toe_link")[0]
    rf_idx = robot.find_bodies("right_toe_link")[0]
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

# ===================================================================
# 9. UNIVERSAL PHYSICS PENALTIES
# ===================================================================
def power_consumption(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    asset = env.scene[asset_cfg.name]
    if asset_cfg.joint_ids is not None:
        torque = asset.data.applied_torque[:, asset_cfg.joint_ids]
        vel = asset.data.joint_vel[:, asset_cfg.joint_ids]
    else:
        torque = asset.data.applied_torque
        vel = asset.data.joint_vel
    power = torch.abs(torque) * torch.abs(vel)
    power = torch.clamp(power, max=5000.0)
    return torch.sum(power, dim=1)

def arm_swing_symmetry(env: ManagerBasedRLEnv, left_arm_cfg: SceneEntityCfg, right_arm_cfg: SceneEntityCfg,
                       left_leg_cfg: SceneEntityCfg, right_leg_cfg: SceneEntityCfg) -> torch.Tensor:
    asset = env.scene[left_arm_cfg.name]
    l_arm_vel = asset.data.joint_vel[:, left_arm_cfg.joint_ids].mean(dim=1)
    r_arm_vel = asset.data.joint_vel[:, right_arm_cfg.joint_ids].mean(dim=1)
    l_leg_vel = asset.data.joint_vel[:, left_leg_cfg.joint_ids].mean(dim=1)
    r_leg_vel = asset.data.joint_vel[:, right_leg_cfg.joint_ids].mean(dim=1)
    
    l_arm_dir = torch.tanh(l_arm_vel)
    r_arm_dir = torch.tanh(r_arm_vel)
    l_leg_dir = torch.tanh(l_leg_vel)
    r_leg_dir = torch.tanh(r_leg_vel)
    
    symmetry_error = torch.square(l_arm_dir - r_leg_dir) + torch.square(r_arm_dir - l_leg_dir)
    return -symmetry_error

# ===================================================================
# 10. EVENTS (RSI, Target Spawning, Style Sampling)
# ===================================================================
def reset_to_reference_pose(env: ManagerBasedRLEnv, env_ids: torch.Tensor) -> None:
    """Reset joint positions and velocities to the first frame of the active mocap clip."""
    data = _ase_data(env)
    ref_pose = data.get("start_joint_pos")
    ref_vel = data.get("start_joint_vel")
    if ref_pose is not None and ref_vel is not None:
        robot = env.scene["robot"]
        if ref_pose.dim() == 2 and ref_pose.shape[0] == 1:
            ref_pose = ref_pose.expand(len(env_ids), -1)
            ref_vel = ref_vel.expand(len(env_ids), -1)
        else:
            ref_pose = ref_pose[env_ids]
            ref_vel = ref_vel[env_ids]
        robot.write_joint_state_to_sim(ref_pose, ref_vel, env_ids=env_ids)

def spawn_target_object(env: ManagerBasedRLEnv, env_ids: torch.Tensor) -> None:
    """Place the target object at a random reachable location for combat/precision."""
    target = env.scene.rigid_objects["target_object"]
    n = len(env_ids)
    pos = torch.zeros((n, 3), device=env.device)
    pos[:, 0] = 0.3 + 0.8 * torch.rand(n, device=env.device)
    pos[:, 1] = (torch.rand(n, device=env.device) - 0.5) * 1.0
    pos[:, 2] = 0.5 + 0.6 * torch.rand(n, device=env.device)
    target.write_root_pose_to_sim(pos, env_ids=env_ids)

def apply_weapon_physics(env: ManagerBasedRLEnv, env_ids: torch.Tensor) -> None:
    """Places the weapon prop in the robot's right hand."""
    weapon = env.scene.rigid_objects["weapon_prop"]
    robot = env.scene["robot"]
    palm_idx = robot.find_bodies("rh_hand_palm")[0]
    palm_pos = robot.data.body_pos_w[env_ids, palm_idx, :]
    weapon.write_root_pose_to_sim(palm_pos, env_ids=env_ids)

def sample_ase_style(env: ManagerBasedRLEnv, env_ids: torch.Tensor) -> None:
    """Trigger motion library sampling on environment reset (Phase 2)."""
    if hasattr(env, "motion_library_manager"):
        env.motion_library_manager.sample(env, env_ids)

# ===================================================================
# 11. DISCRIMINATOR TASK-SPACE BIAS (training-loop utility)
# ===================================================================
def discriminator_task_space_bias(state: torch.Tensor, scale_factors: torch.Tensor) -> torch.Tensor:
    """Scale end-effector and CoM components to bias the discriminator."""
    return state * scale_factors.unsqueeze(0)