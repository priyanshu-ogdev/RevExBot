# C:\Projects\RevExBot\revex_ext\assets\robot.py

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.actuators import ImplicitActuatorCfg
import os

# Dynamically locate your USD file based on your project root
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
USD_PATH = os.path.join(PROJECT_ROOT, "assets", "usd", "revexbot.usd")

# =====================================================================
# 🚨 THE MASTER HARDWARE BLUEPRINT (Used by ALL RL Phases)
# =====================================================================
REVEX_BOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=USD_PATH,
        
        # Base Physics Properties (Randomization will be applied over these in env_cfg.py)
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=10.0,
        ),
        
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, # Necessary evil to prevent finger mesh explosions
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=1,
        ),
    ),
    
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.85), # Default spawn height to prevent floor-clipping
    ),
    
    # 🚨 HARDWARE LIMITS & SEGMENTATION
    # Groups actuators by their real-world capabilities to prevent solver instability
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_.*", ".*_knee_.*", ".*_ankle_.*", ".*_toe_.*"],
            stiffness=40.0, 
            damping=4.0,
            effort_limit=150.0,  # Capped at strongest knee limit
            velocity_limit=8.0,
        ),
        "torso_neck": ImplicitActuatorCfg(
            joint_names_expr=["waist_.*", "neck_.*"],
            stiffness=30.0, 
            damping=3.0,
            effort_limit=150.0,
            velocity_limit=5.0,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_.*", ".*_elbow_.*", ".*_wrist_.*"],
            stiffness=20.0, 
            damping=2.0,
            effort_limit=80.0,
            velocity_limit=8.0,
        ),
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=[".*_pris_.*"], # Targets the prismatic driving servos
            stiffness=2.0,  
            damping=0.2,
            effort_limit=3.0,    # Strict URDF finger limit
            velocity_limit=0.1,  # Strict URDF linear velocity limit
        ),
    },
)