import torch
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.sensors import ContactSensor

def power_consumption(env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    asset = env.scene[asset_cfg.name]
    power = torch.abs(asset.data.applied_torque) * torch.abs(asset.data.joint_vel)
    return torch.sum(power, dim=1)

def arm_swing_symmetry(env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    asset = env.scene[asset_cfg.name]
    
    try:
        l_shoulder_idx = asset.find_joints(["left_shoulder_pitch_joint"])[0][0]
        r_shoulder_idx = asset.find_joints(["right_shoulder_pitch_joint"])[0][0]
        l_hip_idx = asset.find_joints(["left_hip_pitch_joint"])[0][0]
        r_hip_idx = asset.find_joints(["right_hip_pitch_joint"])[0][0]
    except IndexError:
        return torch.zeros(env.num_envs, device=env.device)

    l_shoulder_vel = torch.tanh(asset.data.joint_vel[:, l_shoulder_idx])
    r_shoulder_vel = torch.tanh(asset.data.joint_vel[:, r_shoulder_idx])
    l_hip_vel = torch.tanh(asset.data.joint_vel[:, l_hip_idx])
    r_hip_vel = torch.tanh(asset.data.joint_vel[:, r_hip_idx])
    
    symmetry_error = torch.square(l_shoulder_vel + r_hip_vel) + torch.square(r_shoulder_vel + l_hip_vel)
    return -symmetry_error

def foot_slip_penalty(env, asset_cfg: SceneEntityCfg, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    asset = env.scene[asset_cfg.name]
    contact_sensor = env.scene.sensors[sensor_cfg.name]
    
    foot_indices = asset.find_bodies(asset_cfg.body_names)[0]
    foot_vel_w = asset.data.body_lin_vel_w[:, foot_indices, :]
    
    net_forces = contact_sensor.data.net_forces_w_history[:, 0, :, :] 
    has_contact = torch.norm(net_forces, dim=-1) > 5.0 
    
    horz_vel = torch.norm(foot_vel_w[:, :, :2], dim=-1)
    return -torch.sum(horz_vel * has_contact.float(), dim=1)

def feet_air_time(env, sensor_cfg: SceneEntityCfg, command_name: str) -> torch.Tensor:
    contact_sensor = env.scene.sensors[sensor_cfg.name]
    command = env.command_manager.get_command(command_name)
    
    is_moving = torch.norm(command[:, :3], dim=-1) > 0.1
    net_forces = contact_sensor.data.net_forces_w_history
    has_contact = torch.norm(net_forces, dim=-1) > 1.0
    contact_sum = torch.sum(has_contact.float(), dim=[1, 2]) 
    
    return -contact_sum * is_moving.float()

def smoothness_penalty(env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    asset = env.scene[asset_cfg.name]
    jerk = torch.diff(asset.data.joint_acc, n=1, dim=0) 
    if jerk.shape[0] == 0:
        return torch.zeros(env.num_envs, device=env.device)
    return -torch.mean(torch.square(jerk), dim=[1, 2])