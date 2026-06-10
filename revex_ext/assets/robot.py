# C:\Projects\RevExBot\revex_ext\assets\robot.py

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.actuators import ImplicitActuatorCfg
import os

# Dynamically locate your USD file based on your project root
# Assuming your USD gets compiled to C:\Projects\RevExBot\assets\usd\revexbot1.usd
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
USD_PATH = os.path.join(PROJECT_ROOT, "assets", "usd", "revexbot1.usd")

# =====================================================================
# 🚨 THE MASTER HARDWARE BLUEPRINT (Used by ALL RL Phases)
# =====================================================================
REVEX_BOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=10.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            # 🚨 MANDATORY FIX: Prevents the 10 fingers from exploding the PhysX solver
            enabled_self_collisions=False, 
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.85), # Default spawn height (adjust if robot clips into floor)
        # We can add default joint positions (q=0) here later if needed
    ),
    actuators={
        # This applies a baseline stiffness/damping to all joints. 
        # You will tune these specific PD gains during the Loco phase.
        "all_motors": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=40.0,
            damping=4.0,
        ),
    },
)