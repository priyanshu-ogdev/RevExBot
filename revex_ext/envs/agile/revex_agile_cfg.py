from dataclasses import dataclass
from omni.isaac.lab.envs import RLEnvCfg
from omni.isaac.lab.sim import SimulationCfg, PhysxCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.managers import RewardTermCfg, ObservationGroupCfg, ObservationTermCfg, EventTermCfg, SceneEntityCfg, CurriculumTermCfg
from omni.isaac.lab.utils import configclass
import omni.isaac.lab.envs.mdp as mdp
import omni.isaac.lab.sim as sim_utils

from .. import custom_mdp
from ..custom_terrain import get_phase2_terrain_generator

@configclass
class RevExCommandsCfg:
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(2.0, 2.0),
        simple_heading=True, # 🔥 Forces robot to align heading with velocity vector
        ranges={
            "lin_vel_x": (-2.5, 2.5), 
            "lin_vel_y": (-1.0, 1.0),
            "ang_vel_z": (-1.5, 1.5),
            "heading": (-3.14, 3.14),
        },
    )

@configclass
class RevExActionsCfg:
    joint_positions = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"], 
        scale=0.25, # Limits action spikes for hardware safety
        use_default_offset=True 
    )

@configclass
class RevExRewardsCfg:
    # 1. Agile Tracking
    tracking_lin_vel = RewardTermCfg(func=mdp.track_lin_vel_xy_exp, weight=2.0, params={"std": 0.25})
    tracking_ang_vel = RewardTermCfg(func=mdp.track_ang_vel_z_exp, weight=1.0, params={"std": 0.25})
    heading_alignment = RewardTermCfg(func=mdp.heading_command_exp, weight=0.5, params={"std": 0.25})
    alive_bonus = RewardTermCfg(func=mdp.is_alive, weight=0.1)
    
    # 2. Safety & Smoothness
    action_rate_penalty = RewardTermCfg(func=mdp.action_rate_l2, weight=-0.1)
    joint_accel_penalty = RewardTermCfg(func=mdp.joint_accel_l2, weight=-0.02)
    jerk_penalty = RewardTermCfg(func=custom_mdp.smoothness_penalty, weight=-0.01, params={"asset_cfg": SceneEntityCfg("robot")})
    energy_cost = RewardTermCfg(func=custom_mdp.power_consumption, weight=-0.0005, params={"asset_cfg": SceneEntityCfg("robot")})
    joint_limits = RewardTermCfg(func=mdp.joint_pos_limits, weight=-0.1, params={"asset_cfg": SceneEntityCfg("robot")})
    
    # 🔥 FIX: Prevent the jumping exploit
    vertical_motion_penalty = RewardTermCfg(func=mdp.lin_vel_z_l2, weight=-0.5, params={"asset_cfg": SceneEntityCfg("robot")})
    
    # 3. Humanistic Features
    head_stability = RewardTermCfg(func=mdp.body_ang_vel_l2, weight=-0.5, params={"asset_cfg": SceneEntityCfg("robot", body_names=["head_link"])})
    arm_swing = RewardTermCfg(func=custom_mdp.arm_swing_symmetry, weight=0.2, params={"asset_cfg": SceneEntityCfg("robot")})
    feet_air_time = RewardTermCfg(func=custom_mdp.feet_air_time, weight=0.5, params={"sensor_cfg": SceneEntityCfg("contact_forces"), "command_name": "base_velocity"})
    foot_slip = RewardTermCfg(func=custom_mdp.foot_slip_penalty, weight=-0.5, params={"asset_cfg": SceneEntityCfg("robot", body_names=[".*ankle_roll_link"]), "sensor_cfg": SceneEntityCfg("contact_forces")})
    torso_upright = RewardTermCfg(func=mdp.body_projected_gravity_l2, weight=-1.0, params={"asset_cfg": SceneEntityCfg("robot", body_names=["pelvis_link"])})

@configclass
class RevExObservationsCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        # Actor obs with injected sensor noise and bias
        projected_gravity = ObservationTermCfg(func=mdp.projected_gravity_b, noise=mdp.add_uniform_noise, noise_params={"range": [-0.02, 0.02]})
        joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel, noise=mdp.add_uniform_noise, noise_params={"range": [-0.01, 0.01]})
        joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel, noise=mdp.add_uniform_noise, noise_params={"range": [-0.1, 0.1]})
        actions = ObservationTermCfg(func=mdp.last_action)
        imu_lin_acc = ObservationTermCfg(func=mdp.imu_lin_acc_b, params={"sensor_cfg": SceneEntityCfg("imu_sensor")}, noise=mdp.add_gaussian_noise, noise_params={"mean": 0.0, "std": 0.05})
        imu_ang_vel = ObservationTermCfg(func=mdp.imu_ang_vel_b, params={"sensor_cfg": SceneEntityCfg("imu_sensor")}, noise=mdp.add_gaussian_noise, noise_params={"mean": 0.0, "std": 0.05})
        velocity_commands = ObservationTermCfg(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        # Sparse Exteroception
        height_scan = ObservationTermCfg(func=mdp.height_scan, params={"sensor_cfg": SceneEntityCfg("height_scanner")})

    @configclass
    class CriticCfg(ObservationGroupCfg):
        # Privileged God-Mode
        joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel)
        joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel)
        projected_gravity = ObservationTermCfg(func=mdp.projected_gravity_b)
        actions = ObservationTermCfg(func=mdp.last_action)
        velocity_commands = ObservationTermCfg(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        height_scan = ObservationTermCfg(func=mdp.height_scan, params={"sensor_cfg": SceneEntityCfg("height_scanner")})
        
        true_base_lin_vel = ObservationTermCfg(func=mdp.base_lin_vel_b)
        true_base_ang_vel = ObservationTermCfg(func=mdp.base_ang_vel_b)
        friction_coeffs = ObservationTermCfg(func=mdp.body_friction_coeffs, params={"asset_cfg": SceneEntityCfg("robot")})
        external_force = ObservationTermCfg(func=mdp.external_force_torque_w, params={"asset_cfg": SceneEntityCfg("robot")})

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()

@configclass
class RevExEventsCfg:
    randomize_friction = EventTermCfg(func=mdp.randomize_rigid_body_material, mode="reset", params={"asset_cfg": SceneEntityCfg("robot", body_names=".*"), "static_friction_range": [0.4, 1.5], "operation": "scale"})
    randomize_mass = EventTermCfg(func=mdp.randomize_rigid_body_mass, mode="reset", params={"asset_cfg": SceneEntityCfg("robot", body_names=".*"), "mass_distribution_params": [0.8, 1.2], "operation": "scale"})
    # Hardware Imperfections
    randomize_actuator_gains = EventTermCfg(func=mdp.randomize_actuator_gains, mode="interval", interval_range_s=(2.0, 6.0), params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*"), "stiffness_distribution_params": [0.7, 1.0], "damping_distribution_params": [0.7, 1.0], "operation": "scale"})
    push_robot = EventTermCfg(func=mdp.apply_external_force_torque, mode="interval", interval_range_s=(3.0, 8.0), params={"asset_cfg": SceneEntityCfg("robot", body_names="pelvis_link"), "force_range": (-60.0, 60.0), "torque_range": (-15.0, 15.0)})

@configclass
class RevExCurriculumCfg:
    velocity_ranges = CurriculumTermCfg(func=mdp.modify_command_range, mode="interval", params={"command_name": "base_velocity", "parameter": "lin_vel_x", "min": -0.5, "max": 0.5, "final_min": -2.5, "final_max": 2.5, "num_steps": 20000})

@configclass
class RevExSceneCfg(InteractiveSceneCfg):
    num_envs = 2048
    env_spacing = 2.0
    
    terrain = mdp.TerrainImporterCfg(
        prim_path="/World/ground", terrain_type="generator",
        terrain_generator=get_phase2_terrain_generator(), debug_vis=False,
    )
    
    robot = mdp.ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path="assets/usd/revexbot.usd",
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False, retain_accelerations=False,
                linear_damping=0.0, angular_damping=0.0,
                max_linear_velocity=1000.0, max_angular_velocity=1000.0,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True, # Mandatory for complex terrain recovery
                solver_position_iteration_count=12,
                solver_velocity_iteration_count=4,
            ),
        ),
        init_state=mdp.ArticulationCfg.InitialStateCfg(
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

    contact_forces = mdp.ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*_toe_link|{ENV_REGEX_NS}/Robot/.*ankle_roll_link", history_length=3, filter_prim_paths_expr=["{ENV_REGEX_NS}/Robot"])
    imu_sensor = mdp.ImuSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/imu_in_pelvis")
    
    # 🔥 Sparse 5x5 raycast grid (25 rays total) to prevent VRAM bottlenecks
    # 🔥 Sparse 5x5 raycast grid optimized for VRAM and Compute
    height_scanner = mdp.RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/pelvis_link",
        offset=mdp.RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.0)),
        attach_yaw_only=True,
        pattern_cfg=mdp.GridPatternCfg(resolution=0.2, size=[1.0, 1.0]),
        max_distance=1.5, # 🔥 FIX: Stops PhysX from calculating infinite rays
        debug_vis=False,
    )

@configclass
class RevExAgileCfg(RLEnvCfg):
    seed: int = 42
    episode_length_s: float = 10.0 
    
    sim: SimulationCfg = SimulationCfg(
        dt=0.005, substeps=4, use_gpu_pipeline=True,
        physx=PhysxCfg(
            bounce_threshold_velocity=0.2,
            gpu_max_rigid_contact_count=2**24,
            gpu_max_rigid_patch_count=2**24
        )
    )
    
    scene: RevExSceneCfg = RevExSceneCfg()
    actions: RevExActionsCfg = RevExActionsCfg()
    commands: RevExCommandsCfg = RevExCommandsCfg()
    curriculum: RevExCurriculumCfg = RevExCurriculumCfg()
    rewards: RevExRewardsCfg = RevExRewardsCfg()
    events: RevExEventsCfg = RevExEventsCfg()
    observations: RevExObservationsCfg = RevExObservationsCfg()
    
    terminations: dict = {
        "base_orientation": {"func": mdp.bad_orientation, "params": {"limit_angle": 0.85}},
        "joint_limits": {"func": mdp.joint_pos_out_of_limit, "params": {"threshold": 0.95}}
    }