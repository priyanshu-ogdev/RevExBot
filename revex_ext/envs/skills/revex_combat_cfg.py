# envs/skills/revex_combat_cfg.py
import os
import json
from dataclasses import MISSING

# Isaac Lab core imports
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.utils import configclass
import omni.isaac.lab.envs.mdp as mdp

# RevEx specific imports
from revex_ext.assets.robot import REVEX_BOT_CFG 
from envs.skills.revex_scene_cfg import RevExCombatSceneCfg
from .. import custom_mdp

@configclass
class RevExCombatObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        """ACTOR: Proprioception + Latent Conditioning + Tactical Awareness (184-Float Vector)"""
        # 🧠 Latent Style Conditioning
        latent_style = ObsTerm(func=custom_mdp.get_latent_style_vector)
        latent_style_delta = ObsTerm(func=custom_mdp.get_latent_style_delta)
        
        # Base Proprioception
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        last_action = ObsTerm(func=mdp.last_action) 
        
        # 🎯 Tactical Spatial Buffers (K=5 nearest threats, 15 floats)
        multi_target_vectors = ObsTerm(func=custom_mdp.get_k_nearest_threat_vectors, params={"k": 5})
        
        # 🎯 Active Target & Physics Tracking
        target_pos = ObsTerm(func=mdp.target_pos_rel, params={"asset_cfg": SceneEntityCfg("target_object")}) 
        target_orient = ObsTerm(func=mdp.target_quat_rel, params={"asset_cfg": SceneEntityCfg("target_object")}) 
        object_lin_vel = ObsTerm(func=mdp.object_lin_vel, params={"asset_cfg": SceneEntityCfg("target_object")}, default_val=0.0) 
        wrist_force = ObsTerm(func=mdp.net_forces_and_torques, params={"sensor_cfg": SceneEntityCfg("wrist_contact_sensor")}, default_val=0.0) 
        
        # 📡 Active 360° Spatial Lidar Array (64 floats)
        wrist_rays = ObsTerm(func=mdp.ray_cast_sensor_distances, params={"sensor_cfg": SceneEntityCfg("spatial_awareness_raycaster")}, default_val=3.0)
        
        # 🚨 FIXED: Added missing 36-float height scan pad to hit exactly 184 floats for MoE routing
        height_scan_pad = ObsTerm(func=custom_mdp.zero_pad_height_scan)
        
        def __post_init__(self):
            self.enable_corruption = True 
            self.concatenate_terms = True
            self.history_length = 5 

    @configclass
    class CriticCfg(ObsGroup):
        """CRITIC: Omniscient State + AMP Reference Alignment"""
        policy_obs = ObsTerm(func=mdp.obs_group, params={"group_name": "policy"})
        root_pos_w = ObsTerm(func=mdp.root_pos_w)
        root_lin_vel_w = ObsTerm(func=mdp.root_lin_vel_w)
        root_ang_vel_w = ObsTerm(func=mdp.root_ang_vel_w)
        
        friction = ObsTerm(func=mdp.friction_coef)
        body_mass = ObsTerm(func=mdp.body_mass)
        
        # AMP Reference Tensors (Matches 128-float retargeter output)
        ref_joint_pos = ObsTerm(func=mdp.amp_reference_joint_pos) 
        ref_root_pos = ObsTerm(func=mdp.amp_reference_root_pos)
        ref_root_vel = ObsTerm(func=custom_mdp.amp_reference_root_vel) 
        ref_joint_vel = ObsTerm(func=mdp.amp_reference_joint_vel)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True
            self.history_length = 0

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()
    
@configclass
class RevExCombatActionsCfg:
    joint_positions = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=1.0,
        use_default_offset=True
    )

@configclass
class RevExCombatEventsCfg:
    @configclass
    class Startup(EventTerm):
        randomize_friction = EventTerm(func=mdp.randomize_rigid_body_friction, mode="startup", params={"asset_cfg": SceneEntityCfg("robot"), "friction_range": (0.4, 1.6)})
        randomize_mass = EventTerm(func=mdp.randomize_rigid_body_mass, mode="startup", params={"asset_cfg": SceneEntityCfg("robot", body_names=[".*"]), "mass_distribution_params": [0.8, 1.2], "operation": "scale"})

    @configclass
    class Reset(EventTerm):
        reset_base = EventTerm(func=mdp.reset_root_state_uniform, mode="reset", params={"asset_cfg": SceneEntityCfg("robot"), "pose_range": {"x": (-0.1, 0.1), "y": (-0.1, 0.1), "yaw": (-3.14, 3.14)}, "velocity_range": {"x": (-1.0, 1.0), "y": (-1.0, 1.0), "z": (-0.5, 0.5), "roll": (-0.5, 0.5), "pitch": (-0.5, 0.5), "yaw": (-1.0, 1.0)}})
        reset_joints = EventTerm(func=mdp.reset_joints_by_offset, mode="reset", params={"asset_cfg": SceneEntityCfg("robot"), "position_range": (-0.2, 0.2), "velocity_range": (-2.0, 2.0)})
        reset_target = EventTerm(func=mdp.reset_target_pos, mode="reset", params={"asset_cfg": SceneEntityCfg("target_object"), "pos_range": {"x": (1.0, 4.0), "y": (-2.0, 2.0), "z": (0.5, 1.8)}})
        
        # 🚨 DYNAMIC PHYSICS & LATENT SAMPLING
        apply_weapon_physics = EventTerm(func=custom_mdp.apply_dynamic_weapon_physics, mode="reset")
        sample_new_style = EventTerm(func=custom_mdp.sample_amp_style, mode="reset")
        sample_virtual_opponent = EventTerm(func=custom_mdp.sample_paired_amp_reference, mode="reset", params={"paired_prob": 0.3})

    @configclass # Fixed @classclass typo
    class Interval(EventTerm):
        push_robot = EventTerm(func=mdp.push_by_setting_velocity, mode="interval", interval_range_s=(2.0, 4.0), params={"asset_cfg": SceneEntityCfg("robot"), "velocity_range": {"x": (-1.5, 1.5), "y": (-1.5, 1.5)}})

@configclass
class RevExCombatRewardsCfg:
    # 1. AMP Kinematic Anchor
    amp_tracking = mdp.RewardTermCfg(func=mdp.amp_phase_reward, weight=2.0)
    
    # 2. Tactical Engagement (Approach Target)
    tactical_engagement = mdp.RewardTermCfg(func=mdp.target_position_l2, params={"asset_cfg": SceneEntityCfg("target_object")}, weight=-0.05)
    
    # 3. Kinematic Intent (CoM Momentum Sync)
    com_momentum_sync = mdp.RewardTermCfg(func=custom_mdp.reference_com_velocity_tracking, weight=0.5)
    
    # 4. Strike Efficacy (Velocity-Aligned Impact)
    strike_impact = mdp.RewardTermCfg(func=custom_mdp.contact_strike_reward, params={"sensor_cfg": SceneEntityCfg("wrist_contact_sensor"), "target_cfg": SceneEntityCfg("target_object")}, weight=0.5)
    
    # 5. Latent Transition Smoothness
    kinematic_blend = mdp.RewardTermCfg(func=custom_mdp.latent_transition_penalty, weight=-0.1)
    
    # 6. Weapon Retention (MoE Prior Sync)
    grip_maintenance = mdp.RewardTermCfg(func=mdp.joint_pos_penalty, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*finger.*", ".*thumb.*"]), "target_pos": 0.8}, weight=-0.5)
    
    # 7. Hardware Survival
    fall_penalty = mdp.RewardTermCfg(func=mdp.root_height_penalty, params={"minimum_height": 0.3}, weight=-200.0)
    dof_pos_limits = mdp.RewardTermCfg(func=mdp.joint_pos_limits_penalty, weight=-10.0)
    dof_torques = mdp.RewardTermCfg(func=mdp.joint_torques_penalty, weight=-0.0005) 
    action_rate = mdp.RewardTermCfg(func=mdp.action_rate_l2, weight=-0.01)

@configclass
class RevExCombatEnvCfg(ManagerBasedRLEnvCfg):
    scene: RevExCombatSceneCfg = RevExCombatSceneCfg(num_envs=16384, env_spacing=2.5)

    def __post_init__(self):
        self.observations = RevExCombatObservationsCfg()
        self.actions = RevExCombatActionsCfg()
        self.events = RevExCombatEventsCfg()
        self.rewards = RevExCombatRewardsCfg()
        
        self.sim.dt = 0.005  
        self.decimation = 6  
        self.is_asymmetric = True
        
        # 🚨 AMP DATA BRIDGE
        self.amp_reference_pool_path = "data/motions/reference_pool.json"
        self.prop_manifest_path = "data/motions/prop_manifest.json"