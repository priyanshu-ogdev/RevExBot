# envs/skills/revex_combat_cfg.py
import math
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
# 🚨 CRITICAL IMPORT: The Centralized Scene Blueprint
from envs.skills.revex_scene_cfg import RevExCombatSceneCfg

@configclass
class RevExCombatObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        """ACTOR: Proprioception + Latency Buffer + TACTICAL WEAPON SENSORS"""
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        last_action = ObsTerm(func=mdp.last_action) 
        
        # 🎯 Tactical Aiming (Real Data - Synced to Precision's Shape)
        # GAP 111 FIX: Added asset_cfg params
        target_pos = ObsTerm(
            func=mdp.target_pos_rel, 
            params={"asset_cfg": SceneEntityCfg("target_object")}
        ) 
        target_orient = ObsTerm(
            func=mdp.target_quat_rel, 
            params={"asset_cfg": SceneEntityCfg("target_object")}
        ) 
        
        # ⚔️ Weapon/Object State 
        object_lin_vel = ObsTerm(
            func=mdp.object_lin_vel, 
            params={"asset_cfg": SceneEntityCfg("target_object")}, 
            default_val=0.0
        ) 
        
        # 🛡️ Force Absorption & Recoil Sensing 
        wrist_force = ObsTerm(
            func=mdp.net_forces_and_torques, 
            params={"sensor_cfg": SceneEntityCfg("wrist_contact_sensor")},
            default_val=0.0
        ) 
        
        # 📡 Spatial Awareness (Obstacle/Shield Avoidance)
        wrist_rays = ObsTerm(
            func=mdp.ray_cast_sensor_distances, 
            params={"sensor_cfg": SceneEntityCfg("wrist_raycaster")},
            default_val=2.0  # Matches Scene max_distance
        )
        
        def __post_init__(self):
            self.enable_corruption = True 
            self.concatenate_terms = True
            self.history_length = 5 

    @configclass
    class CriticCfg(ObsGroup):
        """CRITIC: Omniscient State + AMP Target"""
        policy_obs = ObsTerm(func=mdp.obs_group, params={"group_name": "policy"})
        root_pos_w = ObsTerm(func=mdp.root_pos_w)
        root_lin_vel_w = ObsTerm(func=mdp.root_lin_vel_w)
        root_ang_vel_w = ObsTerm(func=mdp.root_ang_vel_w)
        
        friction = ObsTerm(func=mdp.friction_coef)
        body_mass = ObsTerm(func=mdp.body_mass)
        
        ref_joint_pos = ObsTerm(func=mdp.amp_reference_joint_pos) 
        ref_root_pos = ObsTerm(func=mdp.amp_reference_root_pos)   

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True
            self.history_length = 0

@configclass
class RevExCombatActionsCfg:
    joint_efforts = mdp.JointEffortActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=1.0,
        use_default_offset=True
    )

@configclass
class RevExCombatEventsCfg:
    @configclass
    class Startup(EventTerm):
        randomize_friction = EventTerm(
            func=mdp.randomize_rigid_body_friction,
            mode="startup",
            params={"asset_cfg": SceneEntityCfg("robot"), "friction_range": (0.4, 1.6)}
        )
        randomize_mass = EventTerm(
            func=mdp.randomize_rigid_body_mass,
            mode="startup",
            params={"asset_cfg": SceneEntityCfg("robot", body_names=[".*"]), "mass_distribution": "uniform", "mass_range": (0.8, 1.2)}
        )

    @configclass
    class Reset(EventTerm):
        reset_base = EventTerm(
            func=mdp.reset_root_state_uniform,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"), 
                "pose_range": {"x": (-0.1, 0.1), "y": (-0.1, 0.1), "yaw": (-3.14, 3.14)},
                "velocity_range": {"x": (-1.0, 1.0), "y": (-1.0, 1.0), "z": (-0.5, 0.5), "roll": (-0.5, 0.5), "pitch": (-0.5, 0.5), "yaw": (-1.0, 1.0)}
            }
        )
        reset_joints = EventTerm(
            func=mdp.reset_joints_by_offset,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"), 
                "position_range": (-0.2, 0.2), 
                "velocity_range": (-2.0, 2.0) 
            }
        )
        reset_target = EventTerm(
            func=mdp.reset_target_pos,
            mode="reset",
            params={"asset_cfg": SceneEntityCfg("target_object"), "pos_range": {"x": (1.0, 5.0), "y": (-2.0, 2.0), "z": (0.5, 1.8)}}
        )

    @configclass
    class Interval(EventTerm):
        push_robot = EventTerm(
            func=mdp.push_by_setting_velocity,
            mode="interval",
            interval_range_s=(2.0, 4.0),
            params={"asset_cfg": SceneEntityCfg("robot"), "velocity_range": {"x": (-1.5, 1.5), "y": (-1.5, 1.5)}}
        )

@configclass
class RevExCombatRewardsCfg:
    # 1. AMP Mastery (Kinematic Mimicry)
    amp_tracking = mdp.RewardTermCfg(func=mdp.amp_phase_reward, weight=1.0)
    
    # 2. TACTICAL ENGAGEMENT (Base Bias - Verified MDP)
    # Biases the ROBOT BASE toward the target. AMP handles the hand strike.
    tactical_engagement = mdp.RewardTermCfg(
        func=mdp.target_position_l2, 
        params={"asset_cfg": SceneEntityCfg("target_object")},
        weight=-0.05 # Negative L2 rewards getting CLOSE
    )
    
    # 3. WEAPON RETENTION (The Death Grip)
    grip_maintenance = mdp.RewardTermCfg(
        func=mdp.joint_pos_penalty, 
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*finger.*"]), "target_pos": 0.8}, 
        weight=-0.5
    )
    
    # 4. Hardware Survival 
    fall_penalty = mdp.RewardTermCfg(func=mdp.root_height_penalty, params={"minimum_height": 0.3}, weight=-200.0)
    dof_pos_limits = mdp.RewardTermCfg(func=mdp.joint_pos_limits_penalty, weight=-10.0)
    
    # 5. Explosive Kinetic Power
    dof_torques = mdp.RewardTermCfg(func=mdp.joint_torques_penalty, weight=-0.0005) 
    
    # 6. Smooth Flow
    action_rate = mdp.RewardTermCfg(func=mdp.action_rate_l2, weight=-0.01)

@configclass
class RevExCombatEnvCfg(ManagerBasedRLEnvCfg):
    # 🚨 FIX: Scene declared safely at class level
    scene: RevExCombatSceneCfg = RevExCombatSceneCfg(num_envs=4096, env_spacing=2.5)

    def __post_init__(self):
        self.observations = RevExCombatObservationsCfg()
        self.actions = RevExCombatActionsCfg()
        self.events = RevExCombatEventsCfg()
        self.rewards = RevExCombatRewardsCfg()
        
        self.sim.dt = 0.005  
        self.decimation = 6  
        self.amp_motion_file: str = MISSING 
        self.is_asymmetric = True