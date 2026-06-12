"""
Module 4 – Compiler (build_library.py)
Assembles retargeted JSON clips into the unified_motion_library.json.
Features:
- SLERP Quaternion Interpolation (Physics Safe)
- Skill-Isolated Transition Synthesis
- Continuous Velocity Blending
- Latent Z-Code Generation
"""
import os
import json
import torch
import numpy as np
from pathlib import Path
from typing import List, Dict
import argparse
from collections import defaultdict
from scipy.spatial.transform import Rotation as R, Slerp

# ----------------------------------------------------------------------
# 1. PATH CONFIGURATION
# ----------------------------------------------------------------------
PIPELINE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(PIPELINE_DIR)

RETARGETED_DIR = os.path.join(PROJECT_ROOT, "data", "retargeted")
LIBRARY_OUTPUT = os.path.join(PROJECT_ROOT, "data", "unified_motion_library.json")
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

MOCAP_DIM = 39
LATENT_DIM = 16

# ----------------------------------------------------------------------
# 2. LOAD ENCODER (Dummy class for standalone compilation safety)
# ----------------------------------------------------------------------
try:
    from revex_ext.agents.ase_policy import VariationalMotionEncoder
    encoder = VariationalMotionEncoder(mocap_dim=MOCAP_DIM, latent_dim=LATENT_DIM).to(DEVICE)
    encoder.eval()
except ImportError:
    print("⚠️ Warning: VariationalMotionEncoder not found. Using zeroed latents for compilation.")
    encoder = None

def compute_latent_code(frames: List[dict]) -> List[float]:
    if encoder is None:
        return [0.0] * LATENT_DIM
        
    n = min(10, len(frames))
    joint_positions = [frames[i]["joint_pos"] for i in range(n)]
    if n < 10:
        joint_positions += [joint_positions[-1]] * (10 - n)
        
    window = torch.tensor(joint_positions, dtype=torch.float32, device=DEVICE).unsqueeze(0)
    with torch.no_grad():
        z, _, _ = encoder(window)
    return z.squeeze(0).cpu().tolist()

# ----------------------------------------------------------------------
# 3. KINEMATICS & SLERP INTERPOLATION
# ----------------------------------------------------------------------
def compute_kinematic_signature(frames: List[dict]) -> dict:
    base_vels = np.array([f["base_vel"] for f in frames])
    root_positions = np.array([f["root_pos"] for f in frames])
    contact_schedule = np.array([f["contact_schedule"] for f in frames])

    mean_base_vel_mag = float(np.mean(np.linalg.norm(base_vels, axis=1)))
    vertical_com_disp = float(np.max(root_positions[:, 2]) - np.min(root_positions[:, 2]))
    
    planted = (contact_schedule[:, 0] + contact_schedule[:, 1]) > 0
    mean_contact_duty = float(np.mean(planted))

    return {
        "mean_base_vel_mag": mean_base_vel_mag,
        "mean_vertical_com_disp": vertical_com_disp,
        "mean_contact_duty": mean_contact_duty
    }

def interpolate_frames(frame_a: dict, frame_b: dict, alpha: float) -> dict:
    """🚨 FIX 1 & 3: SLERP for Quaternions, LERP for Velocities"""
    
    # SLERP the Quaternions
    rotations = R.from_quat([frame_a["root_rot"], frame_b["root_rot"]])
    slerp = Slerp([0, 1], rotations)
    interp_rot = slerp([alpha]).as_quat()[0].tolist()

    return {
        "root_pos": (np.array(frame_a["root_pos"]) * (1 - alpha) + np.array(frame_b["root_pos"]) * alpha).tolist(),
        "root_rot": interp_rot, 
        "joint_pos": (np.array(frame_a["joint_pos"]) * (1 - alpha) + np.array(frame_b["joint_pos"]) * alpha).tolist(),
        "joint_vel": (np.array(frame_a["joint_vel"]) * (1 - alpha) + np.array(frame_b["joint_vel"]) * alpha).tolist(),
        "base_vel": (np.array(frame_a["base_vel"]) * (1 - alpha) + np.array(frame_b["base_vel"]) * alpha).tolist(),
        "base_ang_vel": (np.array(frame_a["base_ang_vel"]) * (1 - alpha) + np.array(frame_b["base_ang_vel"]) * alpha).tolist(),
        "contact_schedule": [0, 0, 0, 0], # Airborne/sliding assumption during transition
        "phase": 0.0,
        "stiffness_mult": 1.0,
        "rhythm_beat": 0.0
    }

def generate_transition(clip_a: dict, clip_b: dict, num_frames: int = 10) -> dict:
    last_frame = clip_a["frames"][-1]
    first_frame = clip_b["frames"][0]
    trans_frames = []
    
    for i in range(num_frames):
        alpha = (i + 1) / (num_frames + 1)
        trans_frames.append(interpolate_frames(last_frame, first_frame, alpha))
        
    return {
        "skill_id": f"{clip_a['skill_id']}_to_{clip_b['skill_id']}",
        "skill_type": clip_b["skill_type"], # Inherit the target skill type
        "duration_s": num_frames / 30.0, 
        "frames": trans_frames
    }

# ----------------------------------------------------------------------
# 4. MAIN COMPILATION
# ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_dir", type=str, default=RETARGETED_DIR)
    parser.add_argument("--output", type=str, default=LIBRARY_OUTPUT)
    args = parser.parse_args()

    json_files = sorted(Path(args.input_dir).glob("*.json"))
    if not json_files:
        print(f"❌ No retargeted JSON files found in {args.input_dir}")
        return

    print(f"📚 Compiler started – processing {len(json_files)} clips...")

    clips = []
    for json_path in json_files:
        with open(json_path, "r") as f:
            clip = json.load(f)
        clips.append(clip)

    # 🚨 FIX 2: Group clips by skill_type before synthesizing transitions
    skill_groups = defaultdict(list)
    for c in clips:
        skill_groups[c["skill_type"]].append(c)

    library = {
        "version": "2.0",
        "mocap_dim": MOCAP_DIM,
        "latent_dim": LATENT_DIM,
        "clips": []
    }

    print("🧠 Computing Latents and Signatures...")
    for clip in clips:
        clip["latent_z"] = compute_latent_code(clip["frames"])
        clip["kinematic_signature"] = compute_kinematic_signature(clip["frames"])
        library["clips"].append(clip)

    print("🔗 Synthesizing isolated in-category transitions...")
    transitions = []
    for sk_type, sk_clips in skill_groups.items():
        # Only sort within the isolated skill group
        sk_clips.sort(key=lambda c: c["skill_id"])
        
        for i in range(len(sk_clips) - 1):
            trans = generate_transition(sk_clips[i], sk_clips[i+1])
            trans["latent_z"] = compute_latent_code(trans["frames"])
            trans["kinematic_signature"] = compute_kinematic_signature(trans["frames"])
            transitions.append(trans)
            
    library["clips"].extend(transitions)

    print("💾 Saving Master Library...")
    with open(args.output, "w") as f:
        # Compact formatting to save drive space
        json.dump(library, f, separators=(',', ':'))
        
    print(f"🏁 Compilation complete. Library saved to {args.output}")
    print(f"   Total source clips: {len(clips)}")
    print(f"   Total transitions: {len(transitions)}")
    print(f"   Grand Total: {len(library['clips'])} elements ready for ASE Training.")

if __name__ == "__main__":
    main()