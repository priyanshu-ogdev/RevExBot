# envs/skills/revex_dance_cfg.py
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
# Inherit the exact physical stage from Combat for MoE synchronization
from envs.skills.revex_scene_cfg import RevExCombatSceneCfg
from .. import custom_mdp

@configclass
class RevExDanceObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        """ACTOR: Proprioception + Latent Style + Rhythm/Formation Sync (184-Float Vector)"""
        
        # 🧠 Latent Conditioning (MoE parity)
        latent_style = ObsTerm(func=custom_mdp.get_latent_style_vector)
        latent_style_delta = ObsTerm(func=custom_mdp.get_latent_style_delta)
        
        # Base Proprioception
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        last_action = ObsTerm(func=mdp.last_action) 
        
        # 👥 Formation Tracking (K=5 vectors, 15 floats)
        multi_target_vectors = ObsTerm(func=custom_mdp.get_k_formation_vectors, params={"k": 5})
        
        # 🚨 FIXED: Explicit O(1) Zero-Padding instead of invalid scale properties
        target_pos = ObsTerm(func=custom_mdp.zero_pad_3d) 
        target_orient = ObsTerm(func=custom_mdp.zero_pad_4d) 
        object_lin_vel = ObsTerm(func=custom_mdp.zero_pad_3d)
        wrist_force = ObsTerm(func=custom_mdp.zero_pad_6d)
        
        # 🎵 Rhythm Tracking Module (64 floats)
        wrist_rays = ObsTerm(func=custom_mdp.get_dance_rhythm_array)
        
        # 🚨 FIXED: Added the missing 36-float terrain scanner pad to hit exactly 184 floats
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
class RevExDanceActionsCfg:
    joint_positions = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=1.0,
        use_default_offset=True
    )

@configclass
class RevExDanceEventsCfg:
    @configclass
    class Startup(EventTerm):
        # Friction lowered slightly to allow smooth pivots/spins without snapping
        randomize_friction = EventTerm(func=mdp.randomize_rigid_body_friction, mode="startup", params={"asset_cfg": SceneEntityCfg("robot"), "friction_range": (0.6, 1.0)})
        randomize_mass = EventTerm(func=mdp.randomize_rigid_body_mass, mode="startup", params={"asset_cfg": SceneEntityCfg("robot", body_names=[".*"]), "mass_distribution_params": [0.9, 1.1], "operation": "scale"})

    @configclass
    class Reset(EventTerm):
        reset_base = EventTerm(func=mdp.reset_root_state_uniform, mode="reset", params={"asset_cfg": SceneEntityCfg("robot"), "pose_range": {"x": (-0.1, 0.1), "y": (-0.1, 0.1), "yaw": (-3.14, 3.14)}, "velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "z": (-0.2, 0.2), "roll": (-0.2, 0.2), "pitch": (-0.2, 0.2), "yaw": (-0.5, 0.5)}})
        reset_joints = EventTerm(func=mdp.reset_joints_by_offset, mode="reset", params={"asset_cfg": SceneEntityCfg("robot"), "position_range": (-0.1, 0.1), "velocity_range": (-1.0, 1.0)})
        
        # 🚨 DANCE SPECIFIC LOGIC: Style, Rhythm, and Formation Vectorized Resets
        sample_new_style = EventTerm(func=custom_mdp.sample_amp_style, mode="reset")
        sample_rhythm_offset = EventTerm(func=custom_mdp.sample_beat_phase, mode="reset")
        reset_formation = EventTerm(func=custom_mdp.reset_dance_formation, mode="reset")

    @configclass
    class Interval(EventTerm):
        # Momentum perturbation tests the robot's fluidity recovery
        push_robot = EventTerm(func=mdp.push_by_setting_velocity, mode="interval", interval_range_s=(4.0, 6.0), params={"asset_cfg": SceneEntityCfg("robot"), "velocity_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3)}})

@configclass
class RevExDanceRewardsCfg:
    # 1. AMP Kinematic Anchor (Fluid Mimicry)
    amp_tracking = mdp.RewardTermCfg(func=mdp.amp_phase_reward, weight=1.5)
    
    # 2. Rhythm Synchronization (Aligns joint velocity peaks with musical beat)
    rhythm_sync = mdp.RewardTermCfg(func=custom_mdp.rhythm_synchronization_reward, weight=0.8)
    
    # 3. Formation Harmony (Maintains relative spacing to dynamic ghost team)
    formation_harmony = mdp.RewardTermCfg(func=custom_mdp.formation_harmony_reward, weight=0.4)
    
    # 4. Latent Transition Smoothness (Prevents twitching when VLM changes dance style)
    kinematic_blend = mdp.RewardTermCfg(func=custom_mdp.latent_transition_penalty, weight=-0.1)
    
    # 5. Angular Fluidity (Penalizes jerky, robotic spins)
    angular_fluidity = mdp.RewardTermCfg(func=custom_mdp.angular_fluidity_penalty, weight=-0.05)
    
    # 6. Hardware Survival 
    fall_penalty = mdp.RewardTermCfg(func=mdp.root_height_penalty, params={"minimum_height": 0.3}, weight=-200.0)
    dof_pos_limits = mdp.RewardTermCfg(func=mdp.joint_pos_limits_penalty, weight=-10.0)
    
    # 7. Energy Efficiency (Smoother torques required than combat)
    dof_torques = mdp.RewardTermCfg(func=mdp.joint_torques_penalty, weight=-0.002) 
    action_rate = mdp.RewardTermCfg(func=mdp.action_rate_l2, weight=-0.03)

@configclass
class RevExDanceEnvCfg(ManagerBasedRLEnvCfg):
    # 🚨 ADA SCALE RESTORED
    scene: RevExCombatSceneCfg = RevExCombatSceneCfg(num_envs=16384, env_spacing=2.5)

    def __post_init__(self):
        self.observations = RevExDanceObservationsCfg()
        self.actions = RevExDanceActionsCfg()
        self.events = RevExDanceEventsCfg()
        self.rewards = RevExDanceRewardsCfg()
        
        self.sim.dt = 0.005  
        self.decimation = 6  
        self.is_asymmetric = True
        
        # 🚨 THE DATA BRIDGE
        self.amp_reference_pool_path = "data/motions/reference_pool.json"
        self.prop_manifest_path = "data/motions/prop_manifest.json"