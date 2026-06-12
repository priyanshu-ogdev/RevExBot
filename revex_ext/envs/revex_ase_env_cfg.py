# =============================================================================
# RevEx ASE Unified Environment Configuration
# Phase-aware, skill-conditional, sim-to-real hardened.
# 39-DOF explicitly ordered, split-head synchronized.
# Aligned with latest Isaac Lab API (v1.2+).
# =============================================================================
import math
import torch
from dataclasses import MISSING

from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.sim import SimulationCfg, PhysxCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors import ContactSensorCfg, RayCasterCfg, ImuSensorCfg, patterns
from omni.isaac.lab.assets import ArticulationCfg, RigidObjectCfg
from omni.isaac.lab.sim.spawns.shapes import SphereCfg
from omni.isaac.lab.sim.spawns import UsdFileCfg
from omni.isaac.lab.terrains import TerrainImporterCfg, TerrainGeneratorCfg
from omni.isaac.lab.terrains.config import RoughTerrainCfg
from omni.isaac.lab.managers import (
    RewardTermCfg, ObservationGroupCfg, ObservationTermCfg,
    EventTermCfg, SceneEntityCfg, CurriculumTermCfg, TerminationTermCfg,
)
from omni.isaac.lab.envs.mdp import JointPositionActionCfg

from revex_ext.assets.robot import REVEX_BOT_CFG
from . import custom_mdp
import omni.isaac.lab.envs.mdp as mdp

# ------------------------------------------------------------------
# 1. STANDALONE SCENE (no legacy skill imports)
# ------------------------------------------------------------------
@configclass
class RevExAseSceneCfg(InteractiveSceneCfg):
    """Unified scene: robot, props, terrain, and all sensors (URDF regex aligned)."""
    flat_ground: bool = False

    # ---- Terrain ----
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            size=(8.0, 8.0), border_width=20.0, num_rows=10, num_cols=20,
            horizontal_scale=0.1, vertical_scale=0.005, slope_threshold=0.75,
            use_cache=False,
            sub_terrains={
                "pyramid_stairs": RoughTerrainCfg(size=(2.0, 2.0), step_height_range=(0.05, 0.2)),
                "gaps": RoughTerrainCfg(size=(2.0, 2.0), gap_size_range=(0.1, 0.4)),
            }
        ),
        debug_vis=False,
    )

    # ---- Robot ----
    robot: ArticulationCfg = REVEX_BOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # ---- Interaction props ----
    target_object: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/target_object",
        spawn=SphereCfg(radius=0.1, mass_props={"mass": 1.0}),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(1.0, 0.0, 1.0)),
    )

    weapon_prop: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/WeaponProp",
        spawn=UsdFileCfg(
            usd_path="assets/usd/props/dummy_grip.usd",
            rigid_props={"max_depenetration_velocity": 10.0},
            mass_props={"mass": 0.0},
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, -10.0)),
    )

    # ---- Sensors (URDF aligned) ----
    imu_sensor = ImuSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/pelvis_link")

    foot_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*_toe_link",
        history_length=3,
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Robot"],
        debug_vis=False,
    )

    wrist_contact_sensor = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*h_hand_palm",
        update_period=0.0,
        history_length=3,
        track_air_time=False,
        filter_prim_paths_expr=["/World/ground", "{ENV_REGEX_NS}/target_object", "{ENV_REGEX_NS}/WeaponProp"],
        debug_vis=False,
    )

    spatial_awareness_raycaster = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/pelvis_link",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.0)),
        attach_yaw_only=True,
        pattern_cfg=patterns.CylindricalPatternCfg(resolution=0.1, radius=3.0, num_rays_yaw=64),
        max_distance=3.0,
        debug_vis=False,
        mesh_prim_paths=["/World/ground", "{ENV_REGEX_NS}/target_object"]
    )

    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/pelvis_link",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.0)),
        attach_yaw_only=True,
        pattern_cfg=patterns.GridPatternCfg(resolution=0.2, size=[1.0, 1.0]),
        max_distance=1.5,
        debug_vis=False,
        mesh_prim_paths=["/World/ground"]
    )

    def __post_init__(self):
        if self.flat_ground:
            self.terrain = TerrainImporterCfg(
                prim_path="/World/ground",
                terrain_type="plane",
                terrain_generator=None,
            )

# ------------------------------------------------------------------
# 2. COMMANDS (phase-aware)
# ------------------------------------------------------------------
@configclass
class RevExAseCommandsCfg:
    phase: int = 1

    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(2.0, 2.0),
        simple_heading=True,
        ranges={
            "lin_vel_x": (-2.5, 2.5),
            "lin_vel_y": (-1.0, 1.0),
            "ang_vel_z": (-1.5, 1.5),
            "heading": (-3.14, 3.14),
        },
    )

    style_command = custom_mdp.StyleCommandCfg(
        latent_dim=16,
        motion_library_path="data/unified_motion_library.json",
        resampling_time_range=(4.0, 6.0),
    )

# ------------------------------------------------------------------
# 3. ACTIONS – 39-DOF EXPLICITLY ORDERED (27 Body + 12 Hands)
# ------------------------------------------------------------------
@configclass
class RevExAseActionsCfg:
    joint_positions = JointPositionActionCfg(
        asset_name="robot",
        joint_names=[
            # ----- LEFT LEG (7) -----
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint", "left_toe_joint",
            # ----- RIGHT LEG (7) -----
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint", "right_toe_joint",
            # ----- TORSO / HEAD (3) -----
            "waist_yaw_joint", "neck_yaw_joint", "neck_pitch_joint",
            # ----- LEFT ARM (5) -----
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_yaw_joint",
            # ----- RIGHT ARM (5) -----
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_yaw_joint",
            # ----- LEFT HAND (6) -----
            "lh_pris_index_joint", "lh_pris_middle_joint", "lh_pris_ring_joint",
            "lh_pris_little_joint", "lh_pris_thumb_jo_joint", "lh_pris_thumb_DIP_joint",
            # ----- RIGHT HAND (6) -----
            "rh_pris_index_joint", "rh_pris_middle_joint", "rh_pris_ring_joint",
            "rh_pris_little_joint", "rh_pris_thumb_jo_joint", "rh_pris_thumb_DIP_joint",
        ],
        scale=1.0,
        use_default_offset=True,
    )

# ------------------------------------------------------------------
# 4. REWARDS – all domains, with active weights and SAFE dynamic modulators
# ------------------------------------------------------------------
@configclass
class RevExAseRewardsCfg:
    # ---- Universal locomotion/agility (suppressed during skills) ----
    tracking_lin_vel = RewardTermCfg(func=mdp.track_lin_vel_xy_exp, weight=1.5, params={"asset_cfg": SceneEntityCfg("robot"), "command_name": "base_velocity", "std": 0.25})
    tracking_ang_vel = RewardTermCfg(func=mdp.track_ang_vel_z_exp, weight=0.75, params={"asset_cfg": SceneEntityCfg("robot"), "command_name": "base_velocity", "std": 0.25})
    heading_alignment = RewardTermCfg(func=mdp.track_heading_exp, weight=0.5, params={"asset_cfg": SceneEntityCfg("robot"), "command_name": "base_velocity", "std": 0.25})
    alive_bonus = RewardTermCfg(func=mdp.is_alive, weight=0.1)

    feet_air_time = RewardTermCfg(func=mdp.feet_air_time, weight=0.8, params={
        "sensor_cfg": SceneEntityCfg("foot_contacts"), "command_name": "base_velocity", "threshold": 0.4})
    
    foot_impact_penalty = RewardTermCfg(func=mdp.max_contact_forces_penalty, weight=-0.015, params={
        "sensor_cfg": SceneEntityCfg("foot_contacts"), "max_contact_force": 400.0})

    # ---- Action smoothness (Zoned by hardware groups) ----
    action_rate_legs = RewardTermCfg(func=mdp.action_rate_l2, weight=-0.02, params={
        "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_.*", ".*_knee_.*", ".*_ankle_.*", ".*_toe_joint"])})
    action_rate_torso = RewardTermCfg(func=mdp.action_rate_l2, weight=-0.1, params={
        "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_shoulder_.*", ".*_elbow_.*", ".*_wrist_yaw_joint", "waist_yaw_joint", ".*_neck_.*"])})
    action_rate_hands = RewardTermCfg(func=mdp.action_rate_l2, weight=-0.2, params={
        "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_pris_index_joint", ".*_pris_middle_joint", ".*_pris_ring_joint", ".*_pris_little_joint", ".*_pris_thumb_jo_joint", ".*_pris_thumb_DIP_joint"])})

    # ---- Stability (always active) ----
    base_ang_vel_penalty = RewardTermCfg(func=mdp.base_ang_vel_l2, weight=-0.2, params={"asset_cfg": SceneEntityCfg("robot")})
    torso_upright = RewardTermCfg(func=mdp.body_projected_gravity_l2, weight=-1.0, params={"asset_cfg": SceneEntityCfg("robot", body_names=["pelvis_link"])})

    # ---- Hand & posture (always active) ----
    hand_posture_lock = RewardTermCfg(func=mdp.joint_pos_target_l2, weight=-0.8, params={
        "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_thumb_.*", ".*_index_.*", ".*_middle_.*", ".*_ring_.*", ".*_little_.*"]),
        "target": 0.35})
    arm_swing = RewardTermCfg(func=custom_mdp.arm_swing_symmetry, weight=0.2, params={
        "left_arm_cfg": SceneEntityCfg("robot", joint_names=["left_shoulder_pitch_joint"]),
        "right_arm_cfg": SceneEntityCfg("robot", joint_names=["right_shoulder_pitch_joint"]),
        "left_leg_cfg": SceneEntityCfg("robot", joint_names=["left_hip_pitch_joint"]),
        "right_leg_cfg": SceneEntityCfg("robot", joint_names=["right_hip_pitch_joint"]),
    })

    # ---- Physical limits (always active) ----
    energy_cost = RewardTermCfg(func=custom_mdp.power_consumption, weight=-0.0005, params={"asset_cfg": SceneEntityCfg("robot")})
    joint_limits = RewardTermCfg(func=mdp.joint_pos_limits, weight=-0.1, params={"asset_cfg": SceneEntityCfg("robot")})
    foot_slip = RewardTermCfg(func=mdp.foot_slip, weight=-0.5, params={
        "asset_cfg": SceneEntityCfg("robot", body_names=[".*_toe_link"]),
        "sensor_cfg": SceneEntityCfg("foot_contacts")})
    actuator_saturation = RewardTermCfg(func=mdp.joint_torques_penalty, weight=-0.05, params={"asset_cfg": SceneEntityCfg("robot")})
    knee_compliance = RewardTermCfg(func=mdp.joint_pos_target_l2, weight=0.3, params={
        "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_knee_joint"]), "target": 0.15})

    # ---- Style imitation (Phase 2, active only when skill is active) ----
    style_reward = RewardTermCfg(func=custom_mdp.style_reward, weight=1.0)

    # ---- Combat (active when is_combat_mode) ----
    tactical_engagement = RewardTermCfg(func=mdp.target_position_l2, weight=1.5, params={"asset_cfg": SceneEntityCfg("target_object")})
    strike_impact = RewardTermCfg(func=custom_mdp.contact_strike_reward, weight=0.5, params={
        "sensor_cfg": SceneEntityCfg("wrist_contact_sensor"), "target_cfg": SceneEntityCfg("target_object")})
    com_momentum_sync = RewardTermCfg(func=custom_mdp.reference_com_velocity_tracking, weight=0.1)

    # ---- Dance (active when is_dance_mode) ----
    rhythm_sync = RewardTermCfg(func=custom_mdp.rhythm_synchronization_reward, weight=0.8)
    formation_harmony = RewardTermCfg(func=custom_mdp.formation_harmony_reward, weight=0.4)
    angular_fluidity = RewardTermCfg(func=custom_mdp.angular_fluidity_penalty, weight=-0.05)

    # ---- Precision (active when is_precision_mode) ----
    palm_alignment = RewardTermCfg(func=custom_mdp.palm_alignment_reward, weight=1.0)
    grasp_force_modulation = RewardTermCfg(func=custom_mdp.soft_grasp_impedance_reward, weight=1.0)
    relative_velocity_sync = RewardTermCfg(func=mdp.object_vel_rel, weight=-0.5, params={
        "asset_cfg": SceneEntityCfg("target_object"), "body_cfg": SceneEntityCfg("robot", body_names=[".*h_hand_palm"])})

    # ---- Contact schedule (any skill) ----
    contact_schedule = RewardTermCfg(func=custom_mdp.track_contact_schedule, weight=2.0, params={
        "sensor_cfg": SceneEntityCfg("foot_contacts"), "threshold": 1.0})

    # ---- Physics principles (always active, low weight) ----
    angular_momentum_conservation = RewardTermCfg(
        func=custom_mdp.angular_momentum_conservation_reward, weight=0.1,
        params={"asset_cfg": SceneEntityCfg("robot")})
    com_projection_stability = RewardTermCfg(
        func=custom_mdp.com_projection_stability_reward, weight=0.2,
        params={"asset_cfg": SceneEntityCfg("robot"), "sensor_cfg": SceneEntityCfg("foot_contacts")})

# ------------------------------------------------------------------
# 5. OBSERVATIONS
# ------------------------------------------------------------------
@configclass
class RevExAseObservationsCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        # Core proprioception
        projected_gravity = ObservationTermCfg(func=mdp.projected_gravity, params={"asset_cfg": SceneEntityCfg("robot")}, noise=mdp.add_uniform_noise, noise_params={"range": (-0.02, 0.02)})
        joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("robot")}, noise=mdp.add_uniform_noise, noise_params={"range": (-0.02, 0.02)})
        joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("robot")}, noise=mdp.add_uniform_noise, noise_params={"range": (-0.1, 0.1)})
        last_action = ObservationTermCfg(func=mdp.last_action, noise=mdp.add_uniform_noise, noise_params={"range": (-0.01, 0.01)})
        imu_lin_acc = ObservationTermCfg(func=mdp.imu_lin_acc, params={"sensor_cfg": SceneEntityCfg("imu_sensor")})
        imu_ang_vel = ObservationTermCfg(func=mdp.imu_ang_vel, params={"sensor_cfg": SceneEntityCfg("imu_sensor")})
        velocity_commands = ObservationTermCfg(func=mdp.generated_commands, params={"command_name": "base_velocity"})

        # Heavy noise on base lin vel to force proprioceptive reliance (sim-to-real)
        estimated_base_vel = ObservationTermCfg(
            func=mdp.base_lin_vel,
            params={"asset_cfg": SceneEntityCfg("robot")},
            noise=mdp.add_uniform_noise,
            noise_params={"range": (-0.2, 0.2)}
        )

        height_scan = ObservationTermCfg(
            func=mdp.ray_cast_sensor_distances,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")}
        )

        end_effector_pos = ObservationTermCfg(func=custom_mdp.end_effector_positions)
        skill_phase = ObservationTermCfg(func=custom_mdp.get_encoded_phase)
        style_code = ObservationTermCfg(func=custom_mdp.get_current_style_code)

        interaction_vectors = ObservationTermCfg(func=custom_mdp.get_interaction_vectors, params={"k": 5, "dropout_prob": 0.05})
        auxiliary_sensor = ObservationTermCfg(func=custom_mdp.get_auxiliary_sensor_array, params={"dropout_prob": 0.05})

        target_pos = ObservationTermCfg(func=mdp.target_pos_rel, params={"asset_cfg": SceneEntityCfg("target_object")})
        target_orient = ObservationTermCfg(func=mdp.target_quat_rel, params={"asset_cfg": SceneEntityCfg("target_object")})
        object_lin_vel = ObservationTermCfg(func=mdp.object_lin_vel, params={"asset_cfg": SceneEntityCfg("target_object")})

        wrist_force = ObservationTermCfg(func=mdp.net_forces_and_torques, params={"sensor_cfg": SceneEntityCfg("wrist_contact_sensor")})
        foot_contact = ObservationTermCfg(func=mdp.contact_state, params={"sensor_cfg": SceneEntityCfg("foot_contacts"), "threshold": 1.0})

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
            self.history_length = 10   # feeds the causal CNN

    @configclass
    class CriticCfg(ObservationGroupCfg):
        projected_gravity = ObservationTermCfg(func=mdp.projected_gravity, params={"asset_cfg": SceneEntityCfg("robot")})
        joint_pos = ObservationTermCfg(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("robot")})
        joint_vel = ObservationTermCfg(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("robot")})
        imu_lin_acc = ObservationTermCfg(func=mdp.imu_lin_acc, params={"sensor_cfg": SceneEntityCfg("imu_sensor")})
        imu_ang_vel = ObservationTermCfg(func=mdp.imu_ang_vel, params={"sensor_cfg": SceneEntityCfg("imu_sensor")})
        foot_contact = ObservationTermCfg(func=mdp.contact_state, params={"sensor_cfg": SceneEntityCfg("foot_contacts"), "threshold": 1.0})
        
        true_base_lin_vel = ObservationTermCfg(func=mdp.base_lin_vel, params={"asset_cfg": SceneEntityCfg("robot")})
        true_base_ang_vel = ObservationTermCfg(func=mdp.base_ang_vel, params={"asset_cfg": SceneEntityCfg("robot")})
        friction_coeffs = ObservationTermCfg(func=mdp.body_friction_coeffs, params={"asset_cfg": SceneEntityCfg("robot")})
        style_code = ObservationTermCfg(func=custom_mdp.get_current_style_code)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True
            self.history_length = 0

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()

# ------------------------------------------------------------------
# 6. EVENTS
# ------------------------------------------------------------------
@configclass
class RevExAseEventsCfg:
    randomize_friction = EventTermCfg(func=mdp.randomize_rigid_body_material, mode="reset", params={
        "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
        "static_friction_range": (0.4, 1.5),
        "dynamic_friction_range": (0.4, 1.5),
        "restitution_range": (0.0, 0.5),
        "operation": "scale"
    })
    randomize_mass = EventTermCfg(func=mdp.randomize_rigid_body_mass, mode="reset", params={
        "asset_cfg": SceneEntityCfg("robot", body_names=".*"), "mass_distribution_params": (0.8, 1.2), "operation": "scale"})

    randomize_actuator_gains = EventTermCfg(func=mdp.randomize_actuator_gains, mode="interval", interval_range_s=(2.0, 6.0), params={
        "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
        "stiffness_distribution_params": (0.7, 1.0),
        "damping_distribution_params": (0.7, 1.0), "operation": "scale"})

    push_robot = EventTermCfg(func=mdp.push_by_setting_velocity, mode="interval", interval_range_s=(3.0, 8.0), params={
        "asset_cfg": SceneEntityCfg("robot", body_names=["pelvis_link"]),
        "velocity_range": {"x": (-2.0, 2.0), "y": (-2.0, 2.0)}})

    randomize_finger_friction = EventTermCfg(func=mdp.randomize_rigid_body_material, mode="reset", params={
        "asset_cfg": SceneEntityCfg("robot", body_names=[".*_finger_.*", ".*_thumb_.*"]),
        "static_friction_range": (0.1, 0.4), "dynamic_friction_range": (0.1, 0.4), "restitution_range": (0.0, 0.1), "operation": "override"})

    spawn_target = EventTermCfg(func=custom_mdp.spawn_target_object, mode="reset")
    apply_weapon_physics = EventTermCfg(func=custom_mdp.apply_weapon_physics, mode="reset")

    rsi_pose = EventTermCfg(func=custom_mdp.reset_to_reference_pose, mode="reset")

    sample_style = EventTermCfg(func=custom_mdp.sample_ase_style, mode="reset")

# ------------------------------------------------------------------
# 7. CURRICULUM
# ------------------------------------------------------------------
@configclass
class RevExAseCurriculumCfg:
    velocity_ranges = CurriculumTermCfg(func=mdp.modify_command_range, mode="interval", params={
        "command_name": "base_velocity", "parameter": "lin_vel_x",
        "min": -0.5, "max": 0.5, "final_min": -2.5, "final_max": 2.5,
        "num_steps": 20000
    })
    # Style difficulty is managed internally by the MotionLibraryManager.

# ------------------------------------------------------------------
# 8. TERMINATIONS
# ------------------------------------------------------------------
@configclass
class RevExAseTerminationsCfg:
    base_orientation = TerminationTermCfg(func=mdp.bad_orientation, params={"limit_angle": 0.85})
    joint_limits = TerminationTermCfg(func=mdp.joint_pos_out_of_limit, params={"asset_cfg": SceneEntityCfg("robot"), "threshold": 0.95})
    base_height = TerminationTermCfg(func=mdp.base_height_below_threshold, params={"asset_cfg": SceneEntityCfg("robot"), "threshold": 0.3})

# ------------------------------------------------------------------
# 9. MASTER CONFIG
# ------------------------------------------------------------------
@configclass
class RevExAseEnvCfg(ManagerBasedRLEnvCfg):
    phase: int = 1  # 1 = base loco, 2 = ASE style

    scene: RevExAseSceneCfg = RevExAseSceneCfg(num_envs=8192, env_spacing=2.0)
    actions: RevExAseActionsCfg = RevExAseActionsCfg()
    commands: RevExAseCommandsCfg = RevExAseCommandsCfg(phase=1)
    curriculum: RevExAseCurriculumCfg = RevExAseCurriculumCfg()
    rewards: RevExAseRewardsCfg = RevExAseRewardsCfg()
    events: RevExAseEventsCfg = RevExAseEventsCfg()
    observations: RevExAseObservationsCfg = RevExAseObservationsCfg()
    terminations: RevExAseTerminationsCfg = RevExAseTerminationsCfg()

    style_config: dict = {"latent_dim": 16, "motion_library_path": "data/unified_motion_library.json"}

    def __post_init__(self):
        self.sim = SimulationCfg(
            dt=0.005, substeps=4, use_gpu_pipeline=True,
            physx=PhysxCfg(
                bounce_threshold_velocity=0.2,
                friction_offset_threshold=0.04,
                friction_correlation_distance=0.025,
                gpu_max_rigid_contact_count=2**26,
                gpu_max_rigid_patch_count=2**26,
                enable_stabilization=True,
                gpu_found_lost_pairs_capacity=2**24,
            )
        )
        self.episode_length_s = 8.0
        self.is_asymmetric = True

        # Phase-specific overrides
        if self.phase == 1:
            self.scene.num_envs = 8192
            self.scene.flat_ground = False
            self.curriculum.velocity_ranges.mode = "interval"
            self.commands.phase = 1
        else:  # phase == 2
            self.scene.num_envs = 4096
            self.scene.flat_ground = True
            self.curriculum.velocity_ranges.mode = "none"
            self.commands.phase = 2
            self.events.push_robot.params["velocity_range"] = {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}

        # ---------- Dynamic Reward Modulators ----------
        # Safe .get() wrappers to prevent KeyError in Phase 1
        
        def mod_loco(env, bw):   
            is_skill = env.extras.get("ase_data", {}).get("is_skill_mode", torch.zeros(env.num_envs, device=env.device))
            return bw * (1.0 - is_skill.float())

        def mod_combat(env, bw): 
            is_combat = env.extras.get("ase_data", {}).get("is_combat_mode", torch.zeros(env.num_envs, device=env.device))
            return bw * is_combat.float()

        def mod_dance(env, bw):  
            is_dance = env.extras.get("ase_data", {}).get("is_dance_mode", torch.zeros(env.num_envs, device=env.device))
            return bw * is_dance.float()

        def mod_prec(env, bw):   
            is_prec = env.extras.get("ase_data", {}).get("is_precision_mode", torch.zeros(env.num_envs, device=env.device))
            return bw * is_prec.float()

        def mod_style(env, bw):  
            is_skill = env.extras.get("ase_data", {}).get("is_skill_mode", torch.zeros(env.num_envs, device=env.device))
            return bw * is_skill.float()

        # Attach to loco terms
        for name in ["tracking_lin_vel", "tracking_ang_vel", "heading_alignment", "feet_air_time", "foot_impact_penalty"]:
            getattr(self.rewards, name).modifiers = [mod_loco]
        # Combat
        for name in ["tactical_engagement", "strike_impact", "com_momentum_sync"]:
            getattr(self.rewards, name).modifiers = [mod_combat]
        # Dance
        for name in ["rhythm_sync", "formation_harmony", "angular_fluidity"]:
            getattr(self.rewards, name).modifiers = [mod_dance]
        # Precision
        for name in ["palm_alignment", "grasp_force_modulation", "relative_velocity_sync"]:
            getattr(self.rewards, name).modifiers = [mod_prec]
        # Style (active whenever any skill is active)
        for name in ["style_reward", "contact_schedule"]:
            getattr(self.rewards, name).modifiers = [mod_style]