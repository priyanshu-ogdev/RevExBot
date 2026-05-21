import torch
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.sensors import ContactSensor

def power_consumption(env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize energy cost: torque * velocity."""
    asset = env.scene[asset_cfg.name]
    # Use applied_torque from actuators, not joint_effort (which might be raw physics force)
    power = torch.abs(asset.data.applied_torque) * torch.abs(asset.data.joint_vel)
    return torch.sum(power, dim=1)

def arm_swing_symmetry(env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Reward contralateral arm/leg swing. 
    Left Arm Forward (+) should match Right Leg Forward (+).
    """
    asset = env.scene[asset_cfg.name]
    
    # Find joint indices. Note: find_joints returns a tuple (indices, names)
    try:
        l_shoulder_idx = asset.find_joints(["left_shoulder_pitch_joint"])[0][0]
        r_shoulder_idx = asset.find_joints(["right_shoulder_pitch_joint"])[0][0]
        l_hip_idx = asset.find_joints(["left_hip_pitch_joint"])[0][0]
        r_hip_idx = asset.find_joints(["right_hip_pitch_joint"])[0][0]
    except IndexError:
        # Fallback if regex fails or joints missing
        return torch.zeros(env.num_envs, device=env.device)

    l_shoulder_vel = asset.data.joint_vel[:, l_shoulder_idx]
    r_shoulder_vel = asset.data.joint_vel[:, r_shoulder_idx]
    l_hip_vel = asset.data.joint_vel[:, l_hip_idx]
    r_hip_vel = asset.data.joint_vel[:, r_hip_idx]
    
    # Symmetry: L_Shoulder + R_Hip should be ~0. R_Shoulder + L_Hip should be ~0.
    symmetry_error = torch.square(l_shoulder_vel + r_hip_vel) + \
                     torch.square(r_shoulder_vel + l_hip_vel)
    
    return -symmetry_error

def feet_air_time(env, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Reward distinct stepping motions. 
    Penalizes having both feet on the ground simultaneously for too long (shuffling).
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    
    # Check history of net forces. Shape: [num_envs, history_len, num_bodies, 3]
    net_forces = contact_sensor.data.net_forces_w_history
    
    # Threshold for contact (1.0 N is safe for heavy robots)
    has_contact = torch.norm(net_forces, dim=-1) > 1.0
    
    # Sum contacts over all bodies and time steps. 
    # We want to MINIMIZE this sum (encourage air time).
    contact_sum = torch.sum(has_contact.float(), dim=[1, 2]) 
    
    return -contact_sum # Negative because we want to reward less contact time