# envs/skills/revex_precision_cfg.py
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
from envs.skills.revex_scene_cfg import RevExCombatSceneCfg
from .. import custom_mdp

@configclass
class RevExPrecisionObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        """ACTOR: Proprioception + Latent Style + 6D Tracking + Force Modulation"""
        # 🧠 Latent Conditioning (MoE parity with Combat/Dance)
        latent_style = ObsTerm(func=custom_mdp.get_latent_style_vector)
        latent_style_delta = ObsTerm(func=custom_mdp.get_latent_style_delta)
        
        # Base Proprioception & Latency Bridge
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        last_action = ObsTerm(func=mdp.last_action)
        
        # 🎯 6D Target Tracking
        target_pos = ObsTerm(func=mdp.target_pos_rel, params={"asset_cfg": SceneEntityCfg("target_object")}) 
        target_orient = ObsTerm(func=mdp.target_quat_rel, params={"asset_cfg": SceneEntityCfg("target_object")}) 
        
        # ⚙️ Object & Force State (With Sim-to-Real Noise Injection)
        object_lin_vel = ObsTerm(func=mdp.object_lin_vel, params={"asset_cfg": SceneEntityCfg("target_object")}, default_val=0.0)
        
        # 🚨 CRITICAL: Force sensor noise for hardware robustness
        wrist_force = ObsTerm(
            func=mdp.net_forces_and_torques, 
            params={"sensor_cfg": SceneEntityCfg("wrist_contact_sensor")},
            default_val=0.0,
            noise=mdp.add_uniform_noise, # Simulates F/T sensor jitter
            noise_params={"range": [-0.5, 0.5]} 
        )
        
        # 📡 Obstacle/Pressure Awareness
        wrist_rays = ObsTerm(func=mdp.ray_cast_sensor_distances, params={"sensor_cfg": SceneEntityCfg("spatial_awareness_raycaster")}, default_val=3.0)
        
        def __post_init__(self):
            self.enable_corruption = True 
            self.concatenate_terms = True
            self.history_length = 5 

    @configclass
    class CriticCfg(ObsGroup):
        """CRITIC: Omniscient State + Zero-Padded AMP (Parity Only)"""
        policy_obs = ObsTerm(func=mdp.obs_group, params={"group_name": "policy"})
        root_pos_w = ObsTerm(func=mdp.root_pos_w)
        root_lin_vel_w = ObsTerm(func=mdp.root_lin_vel_w)
        root_ang_vel_w = ObsTerm(func=mdp.root_ang_vel_w)
        
        friction = ObsTerm(func=mdp.friction_coef)
        body_mass = ObsTerm(func=mdp.body_mass)
        
        # 🚨 PARITY FIX: Zero-padded terms match exact AMP reference shapes
        ref_joint_pos = ObsTerm(func=mdp.zero_term) 
        ref_root_pos = ObsTerm(func=mdp.zero_term)   
        ref_root_vel = ObsTerm(func=mdp.zero_term)
        ref_joint_vel = ObsTerm(func=mdp.zero_term)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True
            self.history_length = 0

@configclass
class RevExPrecisionActionsCfg:
    joint_efforts = mdp.JointEffortActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=1.0,
        use_default_offset=True
    )

@configclass
class RevExPrecisionEventsCfg:
    @configclass
    class Startup(EventTerm):
        randomize_friction = EventTerm(func=mdp.randomize_rigid_body_friction, mode="startup", params={"asset_cfg": SceneEntityCfg("robot"), "friction_range": (0.8, 1.2)})
        randomize_object_mass = EventTerm(func=mdp.randomize_rigid_body_mass, mode="startup", params={"asset_cfg": SceneEntityCfg("target_object", body_names=[".*"]), "mass_distribution_params": [0.05, 1.5], "operation": "scale"})

    @configclass
    class Reset(EventTerm):
        reset_base = EventTerm(func=mdp.reset_root_state_uniform, mode="reset", params={"asset_cfg": SceneEntityCfg("robot"), "pose_range": {"x": (-0.05, 0.05), "y": (-0.05, 0.05), "yaw": (-0.5, 0.5)}, "velocity_range": {"x": (-0.05, 0.05), "y": (-0.05, 0.05), "z": (-0.01, 0.01), "roll": (-0.01, 0.01), "pitch": (-0.01, 0.01), "yaw": (-0.05, 0.05)}})
        reset_joints = EventTerm(func=mdp.reset_joints_by_offset, mode="reset", params={"asset_cfg": SceneEntityCfg("robot"), "position_range": (-0.05, 0.05), "velocity_range": (-0.05, 0.05)})
        reset_target = EventTerm(func=mdp.reset_target_pos, mode="reset", params={"asset_cfg": SceneEntityCfg("target_object"), "pos_range": {"x": (0.3, 0.6), "y": (-0.3, 0.3), "z": (0.5, 0.8)}})
        sample_new_style = EventTerm(func=custom_mdp.sample_amp_style, mode="reset")

    @configclass
    class Interval(EventTerm):
        push_robot = EventTerm(func=mdp.push_by_setting_velocity, mode="interval", interval_range_s=(4.0, 6.0), params={"asset_cfg": SceneEntityCfg("robot"), "velocity_range": {"x": (-0.1, 0.1), "y": (-0.1, 0.1)}})

@configclass
class RevExPrecisionRewardsCfg:
    # 1. 6D Pose Tracking
    target_position = mdp.RewardTermCfg(func=mdp.target_position_l2, params={"asset_cfg": SceneEntityCfg("target_object")}, weight=2.0)
    target_orientation = mdp.RewardTermCfg(func=mdp.target_orientation_l2, params={"asset_cfg": SceneEntityCfg("target_object")}, weight=1.5)
    
    # 2. Soft-Grasp Impedance Control (Human-like force modulation)
    grasp_force_modulation = mdp.RewardTermCfg(func=custom_mdp.soft_grasp_impedance_reward, weight=1.0)
    
    # 3. Velocity Sync During Carry
    relative_velocity_sync = mdp.RewardTermCfg(func=mdp.object_vel_rel, params={"asset_cfg": SceneEntityCfg("target_object"), "body_cfg": SceneEntityCfg("robot", body_names=[".*_palm"])}, weight=-0.5)
    
    # 4. Collision/Pressure Avoidance
    collision_penalty = mdp.RewardTermCfg(func=mdp.contact_force_penalty, params={"sensor_cfg": SceneEntityCfg("wrist_contact_sensor")}, weight=-0.05)
    
    # 5. Hardware Survival & Smoothness
    fall_penalty = mdp.RewardTermCfg(func=mdp.root_height_penalty, params={"minimum_height": 0.3}, weight=-200.0)
    dof_pos_limits = mdp.RewardTermCfg(func=mdp.joint_pos_limits_penalty, weight=-10.0)
    dof_torques = mdp.RewardTermCfg(func=mdp.joint_torques_penalty, weight=-0.01) 
    action_rate = mdp.RewardTermCfg(func=mdp.action_rate_l2, weight=-0.05)

@configclass
class RevExPrecisionEnvCfg(ManagerBasedRLEnvCfg):
    # 🚨 ADA SCALE: Matches Combat & Dance exactly
    scene: RevExCombatSceneCfg = RevExCombatSceneCfg(num_envs=16384, env_spacing=2.5)

    def __post_init__(self):
        self.observations = RevExPrecisionObservationsCfg()
        self.actions = RevExPrecisionActionsCfg()
        self.events = RevExPrecisionEventsCfg()
        self.rewards = RevExPrecisionRewardsCfg()
        
        self.sim.dt = 0.005  
        self.decimation = 6  
        self.is_asymmetric = True
        
        self.amp_reference_pool_path = "data/motions/reference_pool.json"
        self.prop_manifest_path = "data/motions/prop_manifest.json"