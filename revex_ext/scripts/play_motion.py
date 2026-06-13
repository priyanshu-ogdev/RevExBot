"""
Ground-Truth Motion Playback – replays a clip from the motion library
using native PhysX position targets to verify kinematic purity.
"""
import argparse
import torch
from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--library", type=str, default="data/unified_motion_library.json")
parser.add_argument("--skill_id", type=str, required=True)
parser.add_argument("--headless", action="store_true", default=False)
parser.add_argument("--loop", action="store_true", help="Loop the animation continuously")
AppLauncher.add_app_launcher_args(parser)
args, _ = parser.parse_known_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

from omni.isaac.lab.envs import ManagerBasedRLEnv
from revex_ext.envs.revex_ase_env_cfg import RevExAseEnvCfg
from revex_ext.pipeline.motion_library_manager import MotionLibraryManager

# 1. Setup environment (Phase 1 – no style)
cfg = RevExAseEnvCfg(phase=1)
cfg.scene.num_envs = 1
env = ManagerBasedRLEnv(cfg=cfg)
env.reset()

# 2. Load the motion library
mgr = MotionLibraryManager(args.library, device=env.device)

clip_idx = None
for i, clip in enumerate(mgr.clips):
    if clip.get("skill_id") == args.skill_id:
        clip_idx = i
        break
if clip_idx is None:
    raise ValueError(f"Skill '{args.skill_id}' not found in library.")

clip = mgr.clips[clip_idx]
frames = clip["frames"]

# 🚨 FIX 3: Prevent divide-by-zero on malformed clips
clip_duration = max(clip["duration_s"], 0.01) 
fps = len(frames) / clip_duration

print(f"🎬 Playing '{args.skill_id}'")
print(f"   Frames: {len(frames)} | Duration: {clip_duration:.2f}s | FPS: {fps:.1f}")

# 3. Time Dilation Fix
robot = env.scene["robot"]
sim_dt = cfg.sim.dt
steps_per_frame = max(1, int((1.0 / fps) / sim_dt))
print(f"   Syncing clocks: {steps_per_frame} physics steps per video frame.")

# 4. Playback loop – Native PhysX Tracking
try:
    while True:
        for frame in frames:
            target_pos = torch.tensor(frame["joint_pos"], device=env.device).unsqueeze(0)
            
            # Set the absolute joint targets for the implicit PD actuators
            robot.set_joint_position_target(target_pos)
            
            # Hold the target while the physics engine catches up to the video framerate
            for _ in range(steps_per_frame):
                robot.write_data_to_sim()
                # 🚨 FIX 1: render=True prevents the Isaac Sim GUI from freezing
                env.sim.step(render=True) 
                robot.update(sim_dt)
                
            # 🚨 FIX 2: Force the SensorManager to update contact/IMU data
            env.scene.update(sim_dt)

        if not args.loop:
            break
except KeyboardInterrupt:
    pass

print("🏁 Playback complete.")
env.close()
simulation_app.close()