# envs/skills/revex_loco_cfg.py
import math
from dataclasses import MISSING

# Isaac Lab core imports
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.sim import SimulationCfg, PhysxCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.managers import RewardTermCfg, ObservationGroupCfg, ObservationTermCfg, EventTermCfg, SceneEntityCfg, CurriculumTermCfg, TerminationTermCfg
from omni.isaac.lab.utils import configclass

# Asset and Sensor Imports
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.sensors import ContactSensorCfg, ImuSensorCfg
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.sim.spawns import UsdFileCfg

import omni.isaac.lab.envs.mdp as mdp
import omni.isaac.lab.sim as sim_utils

# 🚨 GAP 119 FIX: Import our optimized custom MDP
from .. import custom_mdp 

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
    # 🚨 Effort Control: Perfectly matches Phase 2/3 for weight transfer
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
            "num_steps": 10000             
        }
    )

@configclass
class RevExRewardsCfg:
    # 1. Base Tracking
    tracking_lin_vel = RewardTermCfg(func=mdp.track_lin_vel_xy_exp, weight=1.5, params={"std": 0.25})
    tracking_ang_vel = RewardTermCfg(func=mdp.track_ang_vel_z_exp, weight=0.75, params={"std": 0.25})
    alive_bonus = RewardTermCfg(func=mdp.is_alive, weight=0.1)
    
    # 2. Smoothness & True Mechanical Power
    action_rate_penalty = RewardTermCfg(func=mdp.action_rate_l2, weight=-0.05)
    joint_accel_penalty = RewardTermCfg(func=mdp.joint_accel_l2, weight=-0.01)
    
    # 🚨 GAP 119 FIX: Re-activated True Power Consumption
    energy_cost = RewardTermCfg(
        func=custom_mdp.power_consumption, 
        weight=-0.0005, 
        params={"asset_cfg": SceneEntityCfg("robot")}
    )
    
    # 3. Humanistic Gait Shaping
    knee_compliance = RewardTermCfg(
        func=mdp.joint_pos_target_l2, 
        weight=0.5, 
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_knee_joint"]), "target": 0.15}
    )
    
    # 🚨 GAP 119 FIX: Re-activated Contralateral Arm Swing
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
    
    foot_slip = RewardTermCfg(
        func=mdp.foot_slip,
        weight=-0.2,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=[".*ankle_roll_link"]), 
            "sensor_cfg": SceneEntityCfg("contact_forces"), 
            "threshold": 0.1
        }
    )

@configclass
class RevExObservationsCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        projected_gravity = ObservationTermCfg(func=mdp.projected_gravity, noise=mdp.add_uniform_noise, noise_params={"range": [-0.01, 0.01]})
        joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel, noise=mdp.add_uniform_noise, noise_params={"range": [-0.01, 0.01]})
        joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel, noise=mdp.add_uniform_noise, noise_params={"range": [-0.1, 0.1]})
        actions = ObservationTermCfg(func=mdp.last_action)
        imu_lin_acc = ObservationTermCfg(func=mdp.imu_lin_acc, params={"sensor_cfg": SceneEntityCfg("imu_sensor")})
        imu_ang_vel = ObservationTermCfg(func=mdp.imu_ang_vel, params={"sensor_cfg": SceneEntityCfg("imu_sensor")})
        velocity_commands = ObservationTermCfg(func=mdp.generated_commands, params={"command_name": "base_velocity"})

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
    randomize_friction = EventTermCfg(
        func=mdp.randomize_rigid_body_material,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"), 
            "static_friction_range": [0.7, 1.3], 
            "operation": "scale"
        }
    )
    randomize_mass = EventTermCfg(
        func=mdp.randomize_rigid_body_mass,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"), 
            "mass_distribution_params": [0.9, 1.1], 
            "operation": "scale"
        }
    )

@configclass
class RevExTerminationsCfg:
    base_orientation = TerminationTermCfg(
        func=mdp.bad_orientation, 
        params={"limit_angle": 0.7}
    )
    joint_limits = TerminationTermCfg(
        func=mdp.joint_pos_out_of_limit, 
        params={"asset_cfg": SceneEntityCfg("robot"), "threshold": 0.95}
    )
    base_height = TerminationTermCfg(
        func=mdp.base_height_below_threshold,
        params={"asset_cfg": SceneEntityCfg("robot"), "threshold": 0.3}
    )

@configclass
class RevExSceneCfg(InteractiveSceneCfg):
    num_envs = 2048
    env_spacing = 2.0
    
    terrain = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane")
    
    robot = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=UsdFileCfg(
            usd_path="assets/usd/revexbot.usd",
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.63),
            joint_pos={
                "left_hip_pitch_joint": 0.0,
                "left_knee_joint": 0.3,
                "left_ankle_pitch_joint": -0.15,
                "right_hip_pitch_joint": 0.0,
                "right_knee_joint": 0.3,
                "right_ankle_pitch_joint": -0.15,
                "waist_yaw_joint": 0.0,
                "left_shoulder_pitch_joint": 0.2,
                "left_elbow_joint": 0.5,
                "right_shoulder_pitch_joint": 0.2,
                "right_elbow_joint": 0.5,
                ".*_wrist_.*": 0.0,
                ".*_thumb_.*": 0.0,
                ".*_index_.*": 0.0,
                ".*_middle_.*": 0.0,
                ".*_ring_.*": 0.0,
                ".*_pinky_.*": 0.0,
            },
        ),
        actuators={
            "legs": mdp.ImplicitActuatorCfg(
                joint_names_expr=[".*_hip_.*", ".*_knee_.*", ".*_ankle_.*", "waist_yaw_joint"],
                stiffness=150.0,
                damping=5.0,
                armature=0.01,
            ),
            "arms": mdp.ImplicitActuatorCfg(
                joint_names_expr=[".*_shoulder_.*", ".*_elbow_.*", ".*_wrist_.*"],
                stiffness=40.0,
                damping=1.5,
                armature=0.005,
            ),
            "hands": mdp.ImplicitActuatorCfg(
                joint_names_expr=[".*_thumb_.*", ".*_index_.*", ".*_middle_.*", ".*_ring_.*", ".*_pinky_.*"],
                stiffness=3.0,
                damping=0.15,
                armature=0.001,
            ),
        },
    )

    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*_toe_link|{ENV_REGEX_NS}/Robot/.*ankle_roll_link", 
        history_length=3,
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Robot"]
    )
    imu_sensor = ImuSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/imu_in_pelvis")

@configclass
class RevExLocoCfg(ManagerBasedRLEnvCfg):
    # 🚨 GAP 120 FIX: Declare managers safely at the class level
    scene: RevExSceneCfg = RevExSceneCfg(num_envs=2048, env_spacing=2.0)
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
                gpu_max_rigid_contact_count=2**24,
                gpu_max_rigid_patch_count=2**24
            )
        )
        self.is_asymmetric = True