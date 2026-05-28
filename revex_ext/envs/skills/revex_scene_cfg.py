# envs/skills/revex_scene_cfg.py
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from omni.isaac.lab.assets import ArticulationCfg, RigidObjectCfg
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.sim.spawns.shapes import SphereCfg # GAP 97 FIX: Correct Import

# Import your base robot definition
from revex_ext.assets.robot import REVEX_BOT_CFG 

@configclass
class RevExCombatSceneCfg(InteractiveSceneCfg):
    """The complete physical blueprint of the training arena."""
    
    # 1. Spawn the Terrain (Ground Plane)
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=None,
    )

    # 2. Spawn the Robot
    robot: ArticulationCfg = REVEX_BOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    
    # 3. Spawn the Target (Enemy or Object)
    target_object: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/target_object",
        spawn=SphereCfg(
            radius=0.05, 
            rigid_props={"max_depenetration_velocity": 10.0},
            mass_props={"mass": 0.5},
            visual_material=None, 
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.5)),
    )

    # =====================================================================
    # 🚨 THE TACTICAL SENSORS (Properly Registered to the Scene)
    # =====================================================================
    
    # Force/Torque Sensor bolted to BOTH Palms
    # GAP 107 FIX: Use regex that matches any link ending in '_palm' under Robot
    wrist_contact_sensor = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*/.*_palm", # Recursive match for rh_palm/lh_palm
        update_period=0.0, 
        history_length=0, 
        track_air_time=False,
        debug_vis=False
    )
    
    # 4-Ray Grid Pattern attached to BOTH Palms
    wrist_raycaster = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*/.*_palm", # Recursive match
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.0)),
        attach_yaw_only=False,
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[0.1, 0.1]),
        max_distance=2.0,  # GAP 100 FIX: Prevents NaN gradient explosions
        debug_vis=False,
        # Looks for collisions with the floor and the enemy/object
        mesh_prim_paths=["/World/ground", "{ENV_REGEX_NS}/target_object"] 
    )