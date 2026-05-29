# revex_ext/envs/loco/revex_loco_cfg.py
import math
from dataclasses import MISSING

# Isaac Lab core imports
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.sim import SimulationCfg, PhysxCfg
from omni.isaac.lab.managers import (
    RewardTermCfg, ObservationGroupCfg, ObservationTermCfg, 
    EventTermCfg, SceneEntityCfg, CurriculumTermCfg, TerminationTermCfg
)
from omni.isaac.lab.utils import configclass
import omni.isaac.lab.envs.mdp as mdp

# MoE Parity & Custom MDP Imports
from envs.skills.revex_scene_cfg import RevExCombatSceneCfg
from .. import custom_mdp

@configclass
class RevExLocoSceneCfg(RevExCombatSceneCfg):
    """Inherits all MoE sensors/targets to prevent KeyError, keeps terrain flat."""
    pass

@configclass
class RevExCommandsCfg:
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(5.0, 5.0),
        simple_heading=False,
        ranges={
            "lin_vel_x": (-0.5, 0.5), 
            "lin_vel_y": (-0.2, 0.2),
            "ang_vel_z": (-0.5, 0.5),
        },
    )

@configclass
class RevExActionsCfg:
    joint_efforts = mdp.JointEffortActionCfg(
        asset_name="robot",
        joint_names=[".*"], 
        scale=1.0,         
        use_default_offset=True 
    )

@configclass
class RevExCurriculumCfg:
    velocity_ranges = CurriculumTermCfg(
        func=mdp.modify_command_range, 
        mode="interval",
        params={
            "command_name": "base_velocity",
            "parameter": "lin_vel_x",
            "min": -0.5, "max": 0.5,       
            "final_min": -2.0, "final_max": 2.0, 
            "num_steps": 2500
        }
    )

@configclass
class RevExRewardsCfg:
    # 1. Base Tracking
    tracking_lin_vel = RewardTermCfg(func=mdp.track_lin_vel_xy_exp, weight=1.5, params={"std": 0.25})
    tracking_ang_vel = RewardTermCfg(func=mdp.track_ang_vel_z_exp, weight=0.75, params={"std": 0.25})
    alive_bonus = RewardTermCfg(func=mdp.is_alive, weight=0.1)
    
    # 2. Asymmetric Effort (Lazy Upper Body, Stiff Lower Body)
    action_rate_legs = RewardTermCfg(func=mdp.action_rate_l2, weight=-0.02, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_.*", ".*_knee_.*", ".*_ankle_.*"])})
    action_rate_torso = RewardTermCfg(func=mdp.action_rate_l2, weight=-0.1, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_shoulder_.*", ".*_elbow_.*", "waist_yaw_joint"])})
    action_rate_hands = RewardTermCfg(func=mdp.action_rate_l2, weight=-0.2, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_wrist_.*", ".*_thumb_.*", ".*_index_.*", ".*_middle_.*", ".*_ring_.*", ".*_pinky_.*"])})
    
    # 3. Full-Body Instincts
    base_ang_vel_penalty = RewardTermCfg(func=mdp.base_ang_vel_l2, weight=-0.5, params={"asset_cfg": SceneEntityCfg("robot")})
    hand_posture_lock = RewardTermCfg(func=mdp.joint_pos_target_l2, weight=-0.8, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_thumb_.*", ".*_index_.*", ".*_middle_.*", ".*_ring_.*", ".*_pinky_.*"]), "target": 0.35})
    
    # 4. Hardware Protection & Efficiency
    energy_cost = RewardTermCfg(func=custom_mdp.power_consumption, weight=-0.0005, params={"asset_cfg": SceneEntityCfg("robot")})
    joint_limits = RewardTermCfg(func=mdp.joint_pos_limits, weight=-0.2, params={"asset_cfg": SceneEntityCfg("robot")})
    foot_slip = RewardTermCfg(func=mdp.foot_slip, weight=-0.3, params={"asset_cfg": SceneEntityCfg("robot", body_names=[".*ankle_roll_link"]), "sensor_cfg": SceneEntityCfg("contact_forces"), "threshold": 0.1})
    
    # 5. Gait Shaping
    knee_compliance = RewardTermCfg(func=mdp.joint_pos_target_l2, weight=0.3, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_knee_joint"]), "target": 0.15})
    arm_swing = RewardTermCfg(func=custom_mdp.arm_swing_symmetry, weight=0.2, params={
        "left_arm_cfg": SceneEntityCfg("robot", joint_names=["left_shoulder_pitch_joint"]),
        "right_arm_cfg": SceneEntityCfg("robot", joint_names=["right_shoulder_pitch_joint"]),
        "left_leg_cfg": SceneEntityCfg("robot", joint_names=["left_hip_pitch_joint"]),
        "right_leg_cfg": SceneEntityCfg("robot", joint_names=["right_hip_pitch_joint"]),
    })

@configclass
class RevExObservationsCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        # Base Loco Proprioception
        projected_gravity = ObservationTermCfg(func=mdp.projected_gravity, noise=mdp.add_uniform_noise, noise_params={"range": [-0.02, 0.02]})
        joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel, noise=mdp.add_uniform_noise, noise_params={"range": [-0.02, 0.02]})
        joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel, noise=mdp.add_uniform_noise, noise_params={"range": [-0.1, 0.1]})
        last_action = ObservationTermCfg(func=mdp.last_action, noise=mdp.add_uniform_noise, noise_params={"range": [-0.01, 0.01]})
        
        imu_lin_acc = ObservationTermCfg(func=mdp.imu_lin_acc, params={"sensor_cfg": SceneEntityCfg("imu_sensor")})
        imu_ang_vel = ObservationTermCfg(func=mdp.imu_ang_vel, params={"sensor_cfg": SceneEntityCfg("imu_sensor")})
        velocity_commands = ObservationTermCfg(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        
        # =====================================================================
        # 🚨 UNIVERSAL MOE PARITY SINK (scale=0.0)
        # Zeros out Combat/Dance arrays to guarantee perfect 184-float matrix shape parity
        # =====================================================================
        latent_style = ObservationTermCfg(func=custom_mdp.get_latent_style_vector, scale=0.0)
        latent_style_delta = ObservationTermCfg(func=custom_mdp.get_latent_style_delta, scale=0.0)
        multi_target_vectors = ObservationTermCfg(func=custom_mdp.get_k_nearest_threat_vectors, params={"k": 5}, scale=0.0)
        
        target_pos = ObservationTermCfg(func=mdp.target_pos_rel, params={"asset_cfg": SceneEntityCfg("target_object")}, scale=0.0) 
        target_orient = ObservationTermCfg(func=mdp.target_quat_rel, params={"asset_cfg": SceneEntityCfg("target_object")}, scale=0.0) 
        object_lin_vel = ObservationTermCfg(func=mdp.object_lin_vel, params={"asset_cfg": SceneEntityCfg("target_object")}, default_val=0.0, scale=0.0)
        wrist_force = ObservationTermCfg(func=mdp.net_forces_and_torques, params={"sensor_cfg": SceneEntityCfg("wrist_contact_sensor")}, default_val=0.0, scale=0.0)
        
        # Combat's 360-lidar and height scanners padded to zero
        combat_lidar_pad = ObservationTermCfg(func=mdp.ray_cast_sensor_distances, params={"sensor_cfg": SceneEntityCfg("spatial_awareness_raycaster")}, default_val=3.0, scale=0.0)
        height_scan_pad = ObservationTermCfg(func=mdp.ray_cast_sensor_distances, params={"sensor_cfg": SceneEntityCfg("height_scanner")}, default_val=1.5, scale=0.0)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
            self.history_length = 5 

    @configclass
    class CriticCfg(ObservationGroupCfg):
        policy_obs = ObservationTermCfg(func=mdp.obs_group, params={"group_name": "policy"})
        true_base_lin_vel = ObservationTermCfg(func=mdp.base_lin_vel, params={"asset_cfg": SceneEntityCfg("robot")})
        true_base_ang_vel = ObservationTermCfg(func=mdp.base_ang_vel, params={"asset_cfg": SceneEntityCfg("robot")})
        friction_coeffs = ObservationTermCfg(func=mdp.body_friction_coeffs, params={"asset_cfg": SceneEntityCfg("robot")})

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True
            self.history_length = 0

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()
    
@configclass
class RevExEventsCfg:
    randomize_friction = EventTermCfg(func=mdp.randomize_rigid_body_material, mode="reset", params={"asset_cfg": SceneEntityCfg("robot", body_names=".*"), "static_friction_range": [0.7, 1.3], "operation": "scale"})
    randomize_mass = EventTermCfg(func=mdp.randomize_rigid_body_mass, mode="reset", params={"asset_cfg": SceneEntityCfg("robot", body_names=".*"), "mass_distribution_params": [0.9, 1.1], "operation": "scale"})

@configclass
class RevExTerminationsCfg:
    base_orientation = TerminationTermCfg(func=mdp.bad_orientation, params={"limit_angle": 0.7})
    joint_limits = TerminationTermCfg(func=mdp.joint_pos_out_of_limit, params={"asset_cfg": SceneEntityCfg("robot"), "threshold": 0.95})
    base_height = TerminationTermCfg(func=mdp.base_height_below_threshold, params={"asset_cfg": SceneEntityCfg("robot"), "threshold": 0.3})

@configclass
class RevExLocoCfg(ManagerBasedRLEnvCfg):
    # 🚨 FIXED: Inherits Universal MoE Scene instead of standalone duplicate
    scene: RevExLocoSceneCfg = RevExLocoSceneCfg(num_envs=16384, env_spacing=2.0)
    
    actions: RevExActionsCfg = RevExActionsCfg()
    commands: RevExCommandsCfg = RevExCommandsCfg()
    curriculum: RevExCurriculumCfg = RevExCurriculumCfg()
    rewards: RevExRewardsCfg = RevExRewardsCfg()
    events: RevExEventsCfg = RevExEventsCfg()
    observations: RevExObservationsCfg = RevExObservationsCfg()
    terminations: RevExTerminationsCfg = RevExTerminationsCfg()
    
    def __post_init__(self):
        self.episode_length_s = 5.0 
        self.sim = SimulationCfg(
            dt=0.005, 
            substeps=4, 
            use_gpu_pipeline=True,
            physx=PhysxCfg(
                bounce_threshold_velocity=0.2,
                friction_offset_threshold=0.04,
                friction_correlation_distance=0.025,
                gpu_max_rigid_contact_count=2**26,
                gpu_max_rigid_patch_count=2**26,
                enable_stabilization=True,
                gpu_found_lost_pairs_capacity=2**24
            )
        )
        self.is_asymmetric = True