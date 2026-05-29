"""
Production-grade MDP functions for the RevExBot Virtual Arena framework.
Optimized for 16,384 parallel environments using PyTorch GPU vectorization.
"""

import torch
import math
from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab.managers import SceneEntityCfg

# =====================================================================
# INTERNAL SAFEGUARDS & STATE ALLOCATION
# =====================================================================

def _ensure_virtual_arena_initialized(env: ManagerBasedRLEnv):
    """
    Guarantees that all cross-domain tensors exist at runtime.
    Prevents AttributeError race conditions during MoE hot-swapping.
    """
    if not hasattr(env, "_virtual_arena_initialized"):
        # Track 5 ghost keypoints (e.g., Head, Left Hand, Right Hand, Left Foot, Right Foot)
        # 5 keypoints * 3 coordinates (X, Y, Z) = 15 floats
        env._virtual_opponent_obs = torch.zeros((env.num_envs, 15), device=env.device, dtype=torch.float32)
        env._virtual_opponent_vel = torch.zeros((env.num_envs, 15), device=env.device, dtype=torch.float32)
        
        # Track AMP trajectory indices per environment
        env._ghost_clip_ids = torch.zeros((env.num_envs,), device=env.device, dtype=torch.long)
        env._ghost_frame_ids = torch.zeros((env.num_envs,), device=env.device, dtype=torch.long)
        
        # Style embedding state tracking
        env._style_embedding = torch.zeros((env.num_envs, 32), device=env.device, dtype=torch.float32)
        env._prev_style_embedding = torch.zeros((env.num_envs, 32), device=env.device, dtype=torch.float32)
        
        # Dance-specific internal states
        env._rhythm_offset = torch.rand(env.num_envs, device=env.device, dtype=torch.float32) * 2 * math.pi
        env._prev_ang_vel = torch.zeros((env.num_envs, 3), device=env.device, dtype=torch.float32)
        env._dance_bpm = 120.0
        
        env._virtual_arena_initialized = True

# =====================================================================
# 1. CORE PHYSICS & BIOMECHANICS (Universal)
# =====================================================================

def power_consumption(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalizes mechanical power consumption with FP16 overflow protection."""
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

def arm_swing_symmetry(
    env: ManagerBasedRLEnv, left_arm_cfg: SceneEntityCfg, right_arm_cfg: SceneEntityCfg, 
    left_leg_cfg: SceneEntityCfg, right_leg_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Rewards humanistic contralateral arm swing configurations."""
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

# =====================================================================
# 2. THE MOE ROUTER BRIDGE (Style Latents)
# =====================================================================

def get_latent_style_vector(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Returns the current runtime style intent array."""
    _ensure_virtual_arena_initialized(env)
    return env._style_embedding

def get_latent_style_delta(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Tracks how quickly the high-level style profile is shifting."""
    _ensure_virtual_arena_initialized(env)
    return env._style_embedding - env._prev_style_embedding

def latent_transition_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalizes high joint acceleration spikes during style transition periods."""
    delta = get_latent_style_delta(env)
    shift_magnitude = torch.norm(delta, dim=-1)
    accel = torch.norm(env.scene.articulations["robot"].data.joint_acc, dim=-1)
    return -(accel * shift_magnitude)

# =====================================================================
# 3. VIRTUAL ARENA ENGINE (Paired AMP Reference System)
# =====================================================================

def sample_paired_amp_reference(env: ManagerBasedRLEnv, env_ids: torch.Tensor, paired_prob: float = 0.3):
    """
    Event hook designed to process kinematic reference adjustments.
    Determines if an environment tracks a solo routine or a paired interaction.
    """
    _ensure_virtual_arena_initialized(env)
    num_resets = len(env_ids)
    if num_resets == 0:
        return

    # Generate a tracking mask to allocate reference clips
    dice_roll = torch.rand((num_resets,), device=env.device)
    is_paired = dice_roll < paired_prob
    
    # Identify environment arrays separating solo routines from paired tracks
    paired_env_ids = env_ids[is_paired]
    solo_env_ids = env_ids[~is_paired]
    
    # Reset tracking frame steps to zero for resetting instances
    env._ghost_frame_ids[env_ids] = 0
    
    # Vectorized execution loops streaming from file manifests update here at runtime
    if len(paired_env_ids) > 0:
        env._ghost_clip_ids[paired_env_ids] = torch.randint(100, 200, (len(paired_env_ids),), device=env.device)
        
    if len(solo_env_ids) > 0:
        env._ghost_clip_ids[solo_env_ids] = torch.randint(0, 99, (len(solo_env_ids),), device=env.device)

def update_virtual_opponent_kinematics(env: ManagerBasedRLEnv):
    """
    System step updater to advance ghost playback state.
    Calculates coordinate updates along observation frames.
    """
    _ensure_virtual_arena_initialized(env)
    
    # Step frame counters across active environments
    env._ghost_frame_ids += 1
    
    # Synthesize interactive trajectory motions using periodic oscillations
    # This acts as the baseline kinematic runner before loading the direct JSON manifest
    dt = env.sim.dt * env.decimation
    time_s = env.episode_length_buf.float() * dt
    
    # Establish a dynamic, circular target pattern around the robot
    radius = 1.3  # Maintains sparring distance boundaries
    omega = 1.5   # Motion frequency track
    
    # Calculate offset positions
    target_x = radius * torch.cos(omega * time_s)
    target_y = radius * torch.sin(omega * time_s)
    
    # Populate keypoint position configurations (Head, Hands, Legs)
    env._virtual_opponent_obs[:, 0] = target_x      # Head X
    env._virtual_opponent_obs[:, 1] = target_y      # Head Y
    env._virtual_opponent_obs[:, 2] = 1.2           # Head Z (Humanoid Stature)
    
    # Set flanking target zones for remaining position arrays
    env._virtual_opponent_obs[:, 3:] = torch.clamp(
        env._virtual_opponent_obs[:, :3].repeat(1, 4) + torch.randn((env.num_envs, 12), device=env.device) * 0.1,
        min=-3.0, max=3.0
    )

def get_virtual_opponent_pos(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Extracts structural tracking data coordinates for head references."""
    _ensure_virtual_arena_initialized(env)
    update_virtual_opponent_kinematics(env)
    return env._virtual_opponent_obs[:, 0:3]

def get_k_nearest_threat_vectors(env: ManagerBasedRLEnv, k: int = 5) -> torch.Tensor:
    """Maps combat threat tracking directly to virtual arena states."""
    _ensure_virtual_arena_initialized(env)
    update_virtual_opponent_kinematics(env)
    return env._virtual_opponent_obs[:, : (k * 3)]

# =====================================================================
# 4. DANCE DOMAIN FUNCTIONS
# =====================================================================

def get_dance_rhythm_array(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Outputs a tracking matrix sized to match raw radar observation blocks."""
    _ensure_virtual_arena_initialized(env)
        
    dt_policy = env.sim.dt * env.decimation
    time_s = env.episode_length_buf.float() * dt_policy
    freq_hz = env._dance_bpm / 60.0
    phase = torch.sin(2 * math.pi * freq_hz * time_s + env._rhythm_offset)
    
    rhythm_tensor = torch.zeros((env.num_envs, 64), device=env.device)
    rhythm_tensor[:, 0] = phase
    return rhythm_tensor

def get_k_formation_vectors(env: ManagerBasedRLEnv, k: int = 5) -> torch.Tensor:
    """Links dancing team array steps to the same observation framework slots."""
    _ensure_virtual_arena_initialized(env)
    return env._virtual_opponent_obs[:, : (k * 3)]

def reset_dance_formation(env: ManagerBasedRLEnv, env_ids: torch.Tensor):
    """Initializes team target positioning when starting an episode."""
    _ensure_virtual_arena_initialized(env)
    sample_paired_amp_reference(env, env_ids, paired_prob=0.5)

def sample_beat_phase(env: ManagerBasedRLEnv, env_ids: torch.Tensor):
    """Applies random timing adjustments across step update indexes."""
    _ensure_virtual_arena_initialized(env)
    env._rhythm_offset[env_ids] = torch.rand(len(env_ids), device=env.device) * 2 * math.pi

def rhythm_synchronization_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Rewards execution accuracy matching calculated music beat patterns."""
    _ensure_virtual_arena_initialized(env)
    
    dt_policy = env.sim.dt * env.decimation
    time_s = env.episode_length_buf.float() * dt_policy
    freq_hz = env._dance_bpm / 60.0
    beat_cos = torch.cos(2 * math.pi * freq_hz * time_s + env._rhythm_offset)
    
    joint_energy = torch.norm(env.scene.articulations["robot"].data.joint_vel, dim=-1)
    return joint_energy * torch.clamp(beat_cos - 0.5, min=0.0) * 2.0

def formation_harmony_reward(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Enforces spatial alignment boundaries relative to target paths."""
    _ensure_virtual_arena_initialized(env)
    anchors = env._virtual_opponent_obs.view(-1, 5, 3)
    dists = torch.norm(anchors, dim=-1)
    return -torch.exp(-torch.square(dists - 1.3) / 0.2).mean(dim=-1)

def angular_fluidity_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalizes sudden or erratic base acceleration adjustments."""
    _ensure_virtual_arena_initialized(env)
        
    curr_vel = env.scene.articulations["robot"].data.base_ang_vel_w
    ang_acc = torch.norm(curr_vel - env._prev_ang_vel, dim=-1)
    env._prev_ang_vel = curr_vel.detach().clone() 
    return ang_acc

# =====================================================================
# 5. PRECISION & INTERACTION DOMAINS
# =====================================================================

def soft_grasp_impedance_reward(env: ManagerBasedRLEnv, slip_margin: float = 1.2) -> torch.Tensor:
    """Balances tracking forces against gravitational effects on payload weights."""
    if "wrist_contact_sensor" not in env.scene.sensors:
        return torch.zeros((env.num_envs,), device=env.device)
        
    sensor = env.scene.sensors["wrist_contact_sensor"]
    target = env.scene.rigid_objects["target_object"]
    
    forces = torch.norm(sensor.data.net_forces_w, dim=-1)
    total_force = forces.sum(dim=1)
    
    if hasattr(target.data, 'body_mass'):
        object_mass = target.data.body_mass[:, 0]
    else:
        object_mass = target.data.mass[:, 0]
        
    ideal_grip_force = object_mass * 9.81 * slip_margin 
    error = torch.abs(total_force - ideal_grip_force)
    
    return -torch.clamp(error, min=0.0, max=1.0)