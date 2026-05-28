# envs/skills/revex_agile_cfg.py
import math
from dataclasses import MISSING

# Isaac Lab core imports
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.sim import SimulationCfg, PhysxCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.managers import RewardTermCfg, ObservationGroupCfg, ObservationTermCfg, EventTermCfg, SceneEntityCfg, CurriculumTermCfg, TerminationTermCfg
from omni.isaac.lab.utils import configclass

# 🚨 CORRECTED ASSET/SENSOR IMPORTS
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.sensors import ContactSensorCfg, ImuSensorCfg, RayCasterCfg, patterns
from omni.isaac.lab.terrains import TerrainImporterCfg, TerrainGeneratorCfg
from omni.isaac.lab.terrains.config import RoughTerrainCfg # Standard Import
from omni.isaac.lab.sim.spawns import UsdFileCfg

import omni.isaac.lab.envs.mdp as mdp
import omni.isaac.lab.sim as sim_utils

from .. import custom_mdp

@configclass
class RevExCommandsCfg:
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(2.0, 2.0),
        simple_heading=True, # Forces robot to align heading with velocity vector
        ranges={
            "lin_vel_x": (-2.5, 2.5), 
            "lin_vel_y": (-1.0, 1.0),
            "ang_vel_z": (-1.5, 1.5),
            "heading": (-3.14, 3.14),
        },
    )

@configclass
class RevExActionsCfg:
    joint_efforts = mdp.JointEffortActionCfg( # MoE Sync
        asset_name="robot",
        joint_names=[".*"], 
        scale=1.0, 
        use_default_offset=True 
    )

@configclass
class RevExRewardsCfg:
    # 1. Agile Tracking
    tracking_lin_vel = RewardTermCfg(func=mdp.track_lin_vel_xy_exp, weight=2.0, params={"std": 0.25})
    tracking_ang_vel = RewardTermCfg(func=mdp.track_ang_vel_z_exp, weight=1.0, params={"std": 0.25})
    heading_alignment = RewardTermCfg(func=mdp.track_heading_exp, weight=0.5, params={"std": 0.25})
    alive_bonus = RewardTermCfg(func=mdp.is_alive, weight=0.1)
    
    # 2. Safety & Smoothness
    action_rate_penalty = RewardTermCfg(func=mdp.action_rate_l2, weight=-0.1)
    joint_accel_penalty = RewardTermCfg(func=mdp.joint_accel_l2, weight=-0.02)
    energy_cost = RewardTermCfg(func=custom_mdp.power_consumption, weight=-0.0005, params={"asset_cfg": SceneEntityCfg("robot")})
    joint_limits = RewardTermCfg(func=mdp.joint_pos_limits, weight=-0.1, params={"asset_cfg": SceneEntityCfg("robot")})
    
    # GAP 125 FIX: Removed vertical_motion_penalty. Dynamic movement requires Vz.
    
    # 3. Humanistic Features
    head_stability = RewardTermCfg(func=mdp.body_ang_vel_l2, weight=-0.5, params={"asset_cfg": SceneEntityCfg("robot", body_names=["head_link"])})
    arm_swing = RewardTermCfg(
        func=custom_mdp.arm_swing_symmetry, 
        weight=0.2, 
        params={
            "left_arm_cfg": SceneEntityCfg("robot", joint_names=["left_shoulder_pitch_joint"]),
            "right_arm_cfg": SceneEntityCfg("robot", joint_names=["right_shoulder_pitch_joint"]),
            "left_leg_cfg": SceneEntityCfg("robot", joint_names=["left_hip_pitch_joint"]),
            "right_leg_cfg": SceneEntityCfg("robot", joint_names=["right_hip_pitch_joint"]),
        }
    )
    feet_air_time = RewardTermCfg(func=mdp.feet_air_time, weight=0.5, params={"sensor_cfg": SceneEntityCfg("contact_forces"), "command_name": "base_velocity", "threshold": 0.5})
    foot_slip = RewardTermCfg(func=mdp.foot_slip, weight=-0.5, params={"asset_cfg": SceneEntityCfg("robot", body_names=[".*ankle_roll_link"]), "sensor_cfg": SceneEntityCfg("contact_forces"), "threshold": 0.1})
    torso_upright = RewardTermCfg(func=mdp.body_projected_gravity_l2, weight=-1.0, params={"asset_cfg": SceneEntityCfg("robot", body_names=["pelvis_link"])})

@configclass
class RevExObservationsCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        projected_gravity = ObservationTermCfg(func=mdp.projected_gravity, noise=mdp.add_uniform_noise, noise_params={"range": [-0.02, 0.02]})
        joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel, noise=mdp.add_uniform_noise, noise_params={"range": [-0.01, 0.01]})
        joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel, noise=mdp.add_uniform_noise, noise_params={"range": [-0.1, 0.1]})
        actions = ObservationTermCfg(func=mdp.last_action)
        imu_lin_acc = ObservationTermCfg(func=mdp.imu_lin_acc, params={"sensor_cfg": SceneEntityCfg("imu_sensor")}, noise=mdp.add_gaussian_noise, noise_params={"mean": 0.0, "std": 0.05})
        imu_ang_vel = ObservationTermCfg(func=mdp.imu_ang_vel, params={"sensor_cfg": SceneEntityCfg("imu_sensor")}, noise=mdp.add_gaussian_noise, noise_params={"mean": 0.0, "std": 0.05})
        velocity_commands = ObservationTermCfg(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        
        # Sparse Exteroception
        height_scan = ObservationTermCfg(func=mdp.ray_cast_sensor_distances, params={"sensor_cfg": SceneEntityCfg("height_scanner")})

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObservationGroupCfg):
        joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel)
        joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel)
        projected_gravity = ObservationTermCfg(func=mdp.projected_gravity)
        actions = ObservationTermCfg(func=mdp.last_action)
        velocity_commands = ObservationTermCfg(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        height_scan = ObservationTermCfg(func=mdp.ray_cast_sensor_distances, params={"sensor_cfg": SceneEntityCfg("height_scanner")})
        
        true_base_lin_vel = ObservationTermCfg(func=mdp.base_lin_vel)
        true_base_ang_vel = ObservationTermCfg(func=mdp.base_ang_vel)
        friction_coeffs = ObservationTermCfg(func=mdp.body_friction_coeffs, params={"asset_cfg": SceneEntityCfg("robot")})

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()

@configclass
class RevExEventsCfg:
    randomize_friction = EventTermCfg(func=mdp.randomize_rigid_body_material, mode="reset", params={"asset_cfg": SceneEntityCfg("robot", body_names=".*"), "static_friction_range": [0.4, 1.5], "operation": "scale"})
    randomize_mass = EventTermCfg(func=mdp.randomize_rigid_body_mass, mode="reset", params={"asset_cfg": SceneEntityCfg("robot", body_names=".*"), "mass_distribution_params": [0.8, 1.2], "operation": "scale"})
    randomize_actuator_gains = EventTermCfg(func=mdp.randomize_actuator_gains, mode="interval", interval_range_s=(2.0, 6.0), params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*"), "stiffness_distribution_params": [0.7, 1.0], "damping_distribution_params": [0.7, 1.0], "operation": "scale"})
    
    push_robot = EventTermCfg(func=mdp.push_by_setting_velocity, mode="interval", interval_range_s=(3.0, 8.0), params={"asset_cfg": SceneEntityCfg("robot", body_names=["pelvis_link"]), "velocity_range": {"x": (-1.0, 1.0), "y": (-1.0, 1.0)}})

@configclass
class RevExCurriculumCfg:
    velocity_ranges = CurriculumTermCfg(func=mdp.modify_command_range, mode="interval", params={"command_name": "base_velocity", "parameter": "lin_vel_x", "min": -0.5, "max": 0.5, "final_min": -2.5, "final_max": 2.5, "num_steps": 20000})

@configclass
class RevExTerminationsCfg:
    base_orientation = TerminationTermCfg(func=mdp.bad_orientation, params={"limit_angle": 0.85})
    joint_limits = TerminationTermCfg(func=mdp.joint_pos_out_of_limit, params={"asset_cfg": SceneEntityCfg("robot"), "threshold": 0.95})
    base_height = TerminationTermCfg(func=mdp.base_height_below_threshold, params={"asset_cfg": SceneEntityCfg("robot"), "threshold": 0.3})

@configclass
class RevExSceneCfg(InteractiveSceneCfg):
    num_envs = 2048
    env_spacing = 2.0
    
    # GAP 126 FIX: Define Terrain Inline for Stability
    terrain = TerrainImporterCfg(
        prim_path="/World/ground", 
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            size=(8.0, 8.0),
            border_width=20.0,
            num_rows=10,
            num_cols=20,
            horizontal_scale=0.1,
            vertical_scale=0.005,
            slope_threshold=0.75,
            use_cache=False,
            sub_terrains={
                "pyramid_stairs": RoughTerrainCfg(size=(2.0, 2.0), step_height_range=(0.05, 0.2)),
                "gaps": RoughTerrainCfg(size=(2.0, 2.0), gap_size_range=(0.1, 0.3)),
            }
        ), 
        debug_vis=False,
    )
    
    robot = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=UsdFileCfg(
            usd_path="assets/usd/revexbot.usd",
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False, retain_accelerations=False,
                linear_damping=0.0, angular_damping=0.0,
                max_linear_velocity=1000.0, max_angular_velocity=1000.0,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True, 
                solver_position_iteration_count=12,
                solver_velocity_iteration_count=4,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.65),
            joint_pos={
                "left_hip_pitch_joint": 0.0, "left_knee_joint": 0.3, "left_ankle_pitch_joint": -0.15,
                "right_hip_pitch_joint": 0.0, "right_knee_joint": 0.3, "right_ankle_pitch_joint": -0.15,
                "waist_yaw_joint": 0.0,
                "left_shoulder_pitch_joint": 0.2, "left_elbow_joint": 0.5,
                "right_shoulder_pitch_joint": 0.2, "right_elbow_joint": 0.5,
                ".*_wrist_.*": 0.0, ".*_thumb_.*": 0.0, ".*_index_.*": 0.0, ".*_middle_.*": 0.0, ".*_ring_.*": 0.0, ".*_pinky_.*": 0.0,
            },
        ),
        actuators={
            "legs": mdp.ImplicitActuatorCfg(joint_names_expr=[".*_hip_.*", ".*_knee_.*", ".*_ankle_.*", "waist_yaw_joint"], stiffness=150.0, damping=5.0, armature=0.01, friction_loss=0.1),
            "arms": mdp.ImplicitActuatorCfg(joint_names_expr=[".*_shoulder_.*", ".*_elbow_.*", ".*_wrist_.*"], stiffness=40.0, damping=1.5, armature=0.005, friction_loss=0.05),
            "hands": mdp.ImplicitActuatorCfg(joint_names_expr=[".*_thumb_.*", ".*_index_.*", ".*_middle_.*", ".*_ring_.*", ".*_pinky_.*"], stiffness=3.0, damping=0.15, armature=0.001),
        },
    )

    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*_toe_link|{ENV_REGEX_NS}/Robot/.*ankle_roll_link", history_length=3, filter_prim_paths_expr=["{ENV_REGEX_NS}/Robot"])
    imu_sensor = ImuSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/imu_in_pelvis")
    
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/pelvis_link",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.0)),
        attach_yaw_only=True,
        pattern_cfg=patterns.GridPatternCfg(resolution=0.2, size=[1.0, 1.0]),
        max_distance=1.5, 
        debug_vis=False,
    )

@configclass
class RevExAgileCfg(ManagerBasedRLEnvCfg):
    scene: RevExSceneCfg = RevExSceneCfg(num_envs=2048, env_spacing=2.0)
    actions: RevExActionsCfg = RevExActionsCfg()
    commands: RevExCommandsCfg = RevExCommandsCfg()
    curriculum: RevExCurriculumCfg = RevExCurriculumCfg()
    rewards: RevExRewardsCfg = RevExRewardsCfg()
    events: RevExEventsCfg = RevExEventsCfg()
    observations: RevExObservationsCfg = RevExObservationsCfg()
    terminations: RevExTerminationsCfg = RevExTerminationsCfg()
    
    def __post_init__(self):
        self.seed = 42
        self.episode_length_s = 10.0 
        self.sim = SimulationCfg(
            dt=0.005, substeps=4, use_gpu_pipeline=True,
            physx=PhysxCfg(
                bounce_threshold_velocity=0.2,
                gpu_max_rigid_contact_count=2**24,
                gpu_max_rigid_patch_count=2**24
            )
        )
        self.is_asymmetric = True