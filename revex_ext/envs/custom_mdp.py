# envs/custom_mdp.py
import torch
from omni.isaac.lab.managers import SceneEntityCfg

def power_consumption(env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """
    Penalizes mechanical power consumption.
    Power = Torque * Angular Velocity.
    """
    # Extract the asset using the modern Isaac Lab API
    asset = env.scene[asset_cfg.name]
    
    # If specific joints were passed in the Cfg, use the cached IDs. 
    # Otherwise, use all joints.
    if asset_cfg.joint_ids is not None:
        torque = asset.data.applied_torque[:, asset_cfg.joint_ids]
        vel = asset.data.joint_vel[:, asset_cfg.joint_ids]
    else:
        torque = asset.data.applied_torque
        vel = asset.data.joint_vel
        
    power = torch.abs(torque) * torch.abs(vel)
    return torch.sum(power, dim=1)

def arm_swing_symmetry(
    env, 
    left_arm_cfg: SceneEntityCfg, 
    right_arm_cfg: SceneEntityCfg, 
    left_leg_cfg: SceneEntityCfg, 
    right_leg_cfg: SceneEntityCfg
) -> torch.Tensor:
    """
    Rewards contralateral humanistic arm swing.
    The left arm should swing forward when the right leg swings forward.
    """
    # Extract the robot asset (assumes all cfgs point to the same asset)
    asset = env.scene[left_arm_cfg.name]
    
    # Fast Tensor Slicing using pre-cached joint IDs (No String Matching!)
    l_shoulder_vel = torch.tanh(asset.data.joint_vel[:, left_arm_cfg.joint_ids[0]])
    r_shoulder_vel = torch.tanh(asset.data.joint_vel[:, right_arm_cfg.joint_ids[0]])
    l_hip_vel = torch.tanh(asset.data.joint_vel[:, left_leg_cfg.joint_ids[0]])
    r_hip_vel = torch.tanh(asset.data.joint_vel[:, right_leg_cfg.joint_ids[0]])
    
    # Symmetry Error: 
    # If left shoulder and right hip are moving in opposite directions, sum approaches 0.
    symmetry_error = torch.square(l_shoulder_vel + r_hip_vel) + torch.square(r_shoulder_vel + l_hip_vel)
    
    # Returns negative error (so maximizing the reward pushes error to 0)
    return -symmetry_error