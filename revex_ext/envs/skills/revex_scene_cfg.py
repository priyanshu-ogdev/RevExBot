# envs/skills/revex_scene_cfg.py
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.sensors import ContactSensorCfg, RayCasterCfg, ImuSensorCfg, patterns
from omni.isaac.lab.assets import ArticulationCfg, RigidObjectCfg
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.sim.spawns.shapes import SphereCfg 
from omni.isaac.lab.sim.spawns import UsdFileCfg

from revex_ext.assets.robot import REVEX_BOT_CFG 

@configclass
class RevExCombatSceneCfg(InteractiveSceneCfg):
    """The Universal Physical Blueprint for the RevEx MoE Architecture."""
    
    num_envs = 16384
    env_spacing = 2.5
    
    # 1. Terrain (Defaults to flat plane; Agile will override this)
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=None,
    )

    # 2. Robot
    robot: ArticulationCfg = REVEX_BOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    
    # =====================================================================
    # 🚨 DYNAMIC PROP ARMORY & INTERACTION TARGETS
    # =====================================================================
    weapon_prop: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/WeaponProp",
        spawn=UsdFileCfg(
            usd_path="assets/usd/props/dummy_grip.usd", 
            rigid_props={"max_depenetration_velocity": 10.0},
            mass_props={"mass": 0.0}, # Overwritten at runtime by custom_mdp
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, -10.0)), 
    )

    target_object: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/target_object",
        spawn=SphereCfg(
            radius=0.1, 
            rigid_props={"max_depenetration_velocity": 10.0},
            mass_props={"mass": 1.0},
            visual_material=None, 
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(1.0, 0.0, 1.0)),
    )

    # =====================================================================
    # 🚨 CORE SURVIVAL SENSORS
    # =====================================================================
    imu_sensor = ImuSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/imu_in_pelvis")
    foot_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*_toe_link|{ENV_REGEX_NS}/Robot/.*ankle_roll_link", 
        history_length=3,
        filter_prim_paths_expr=["{ENV_REGEX_NS}/Robot"],
        debug_vis=False
    )

    # =====================================================================
    # 🚨 THE UNIVERSAL SENSOR ARRAY (MoE Parity Guaranteed)
    # =====================================================================
    wrist_contact_sensor = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*_palm", 
        update_period=0.0, 
        history_length=3, 
        track_air_time=False,
        filter_prim_paths_expr=["/World/ground", "{ENV_REGEX_NS}/target_object", "{ENV_REGEX_NS}/WeaponProp"],
        debug_vis=False
    )
    
    # Combat & Dance: 360° Tactical Lidar Proxy
    spatial_awareness_raycaster = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/pelvis_link", 
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.0)),
        attach_yaw_only=True, 
        pattern_cfg=patterns.CylindricalPatternCfg(resolution=0.1, radius=3.0, num_rays_yaw=64),
        max_distance=3.0, 
        debug_vis=False,
        mesh_prim_paths=["/World/ground", "{ENV_REGEX_NS}/target_object"] 
    )

    # Loco & Agile: Downward Terrain Scanner
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/pelvis_link",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.0)),
        attach_yaw_only=True,
        pattern_cfg=patterns.GridPatternCfg(resolution=0.2, size=[1.0, 1.0]),
        max_distance=1.5, 
        debug_vis=False,
        mesh_prim_paths=["/World/ground"] 
    )