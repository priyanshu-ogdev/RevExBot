from omni.isaac.lab.terrains import HfDiscreteObstaclesTerrainCfg

def get_phase2_terrain_generator():
    """Generates realistic curbs, slopes, and gaps for Phase 2 Agility."""
    return HfDiscreteObstaclesTerrainCfg(
        size=(8.0, 8.0),
        border_width=2.0,
        num_rows=10,
        num_cols=10,
        horizontal_scale=0.1,
        vertical_scale=0.02,     # Max 2cm sudden drops to prevent physics clipping
        slope_range=(0.0, 0.35), # Max ~19 degree incline
        platform_width=2.0,
        obstacle_types=["gap", "stair_up", "stair_down"],
        obstacle_height_mode="fixed",
        obstacle_height_range=(0.05, 0.10),
    )