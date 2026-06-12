"""
Module 3 – Retargeter (retarget_urdf.py)
Maps 75 MediaPipe landmarks to 39-DOF RevExBot joint angles.
Features:
- Plane-Based Analytical Kinematics (No Flared Elbows)
- Explicit Ankle Pitch/Roll and Wrist Yaw extraction
- Left/Right Servo Mirroring Logic
- Z-Up, X-Forward Gram-Schmidt Root Alignment
- True Local-Z Contact Extraction & Prismatic Limits
"""
import os
import json
import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as R
from pathlib import Path
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed

# ----------------------------------------------------------------------
# 1. PATH & CONSTANT CONFIGURATION
# ----------------------------------------------------------------------
PIPELINE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(PIPELINE_DIR)
KINEMATICS_DIR = os.path.join(PROJECT_ROOT, "data", "kinematics")
RETARGETED_DIR = os.path.join(PROJECT_ROOT, "data", "retargeted")
os.makedirs(RETARGETED_DIR, exist_ok=True)

ROBOT_LEG_LENGTH = 0.19564 + 0.29466 
PRISMATIC_FINGER_MAX = 0.0103
WEAPON_GRIP = np.array([0.8 * PRISMATIC_FINGER_MAX] * 12)
NEUTRAL_HAND = np.array([0.35 * PRISMATIC_FINGER_MAX] * 12)

# MediaPipe Indices
MP_L_HIP, MP_R_HIP = 23, 24
MP_L_SHLD, MP_R_SHLD = 11, 12
MP_L_ELB, MP_R_ELB = 13, 14
MP_L_WRI, MP_R_WRI = 15, 16
MP_L_KNEE, MP_R_KNEE = 25, 26
MP_L_ANK, MP_R_ANK = 27, 28
MP_L_HEEL, MP_R_HEEL = 29, 30
MP_L_FOOT, MP_R_FOOT = 31, 32
MP_L_INDEX, MP_R_INDEX = 19, 20 # Hand index knuckles

LOWER_LIMITS = np.array([-2.094, -0.785, -0.785, 0.0, -0.35, -0.1, -1.047, -1.0, -0.785, -0.785, -1.5, -0.35, -0.1, 0.0, -1.571, -1.571, -0.785, -3.141, -1.571, -1.571, 0.0, -3.141, -0.872, 0.0, -1.571, -2.443, -3.141, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
UPPER_LIMITS = np.array([1.0, 0.785, 0.785, 1.5, 0.35, 0.1, 0.0, 2.094, 0.785, 0.785, 0.0, 0.35, 0.1, 1.047, 1.571, 1.571, 0.785, 0.872, 0.0, 1.571, 2.443, 3.141, 3.141, 1.571, 1.571, 0.0, 3.141, 0.0103, 0.0103, 0.0103, 0.0103, 0.0103, 0.0103, 0.0103, 0.0103, 0.0103, 0.0103, 0.0103, 0.0103])

MAX_WORKERS = 8

# ----------------------------------------------------------------------
# 2. THE UPGRADED KINEMATICS ENGINE
# ----------------------------------------------------------------------
def compute_root_pose(lm: np.ndarray):
    """Gram-Schmidt Z-Up, X-Forward Root Alignment."""
    lh, rh = lm[MP_L_HIP], lm[MP_R_HIP]
    ls, rs = lm[MP_L_SHLD], lm[MP_R_SHLD]

    pelvis = (lh + rh) / 2.0
    z_up = ((ls + rs) / 2.0) - pelvis
    z_up /= np.linalg.norm(z_up) + 1e-6

    y_left = lh - rh
    y_left -= np.dot(y_left, z_up) * z_up 
    y_left /= np.linalg.norm(y_left) + 1e-6

    x_fwd = np.cross(y_left, z_up)
    x_fwd /= np.linalg.norm(x_fwd) + 1e-6

    return pelvis, np.column_stack([x_fwd, y_left, z_up])

def extract_full_limb_kinematics(joint_a, joint_b, joint_c, extremity, R_parent_inv, is_left_side=False):
    """
    Plane-Based Analytical IK. 
    Solves Pitch, Roll, Yaw, Hinge, and Extremity (Ankle/Wrist) angles.
    """
    # Localize coordinates to Torso Frame
    v_upper = R_parent_inv @ (joint_b - joint_a)
    v_lower = R_parent_inv @ (joint_c - joint_b)
    v_extremity = R_parent_inv @ (extremity - joint_c)

    n_up, n_low = np.linalg.norm(v_upper), np.linalg.norm(v_lower)
    if n_up < 1e-5 or n_low < 1e-5:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    u_upper, u_lower = v_upper / n_up, v_lower / n_low
    
    # 1. Hinge Angle (Knee/Elbow)
    hinge_angle = np.arccos(np.clip(np.dot(u_upper, u_lower), -1.0, 1.0))
    
    # 2. Kinematic Plane Math (Solves Shoulder/Hip Pitch, Roll, Yaw perfectly)
    # The normal of the triangle formed by Shoulder-Elbow-Wrist dictates the roll.
    plane_normal = np.cross(u_upper, u_lower)
    if np.linalg.norm(plane_normal) > 1e-5:
        plane_normal /= np.linalg.norm(plane_normal)
    else:
        plane_normal = np.array([0, 1, 0]) # Fallback if arm is perfectly straight

    try:
        # Create a rotation matrix targeting the upper limb vector and the plane normal
        target_R = np.column_stack([u_upper, plane_normal, np.cross(u_upper, plane_normal)])
        euler = R.from_matrix(target_R).as_euler('xyz', degrees=False)
    except:
        euler = np.array([0.0, 0.0, 0.0])

    # 3. Extremity Flexion (Wrist Yaw / Ankle Pitch)
    u_ext = v_extremity / (np.linalg.norm(v_extremity) + 1e-6)
    extremity_flex = np.arccos(np.clip(np.dot(u_lower, u_ext), -1.0, 1.0))

    # 🚨 FIX: Left/Right Servo Mirroring
    # URDF files generally mirror the Y and Z axes for the left side of the robot.
    mirror = -1.0 if is_left_side else 1.0

    return euler[0], euler[1] * mirror, euler[2] * mirror, hinge_angle, extremity_flex, 0.0

# ----------------------------------------------------------------------
# 3. CORE PROCESSING LOGIC
# ----------------------------------------------------------------------
def apply_padded_savgol(data, window, polyorder):
    pad_len = window
    padded_data = np.pad(data, ((pad_len, pad_len), (0, 0)), mode='reflect')
    smoothed_padded = savgol_filter(padded_data, window, polyorder, axis=0)
    return smoothed_padded[pad_len:-pad_len]

def retarget_clip(npz_path: str) -> dict:
    data = np.load(npz_path, allow_pickle=True)
    lm_seq = data["joint_positions"] 
    fps = data["fps"].item()
    clip_name = str(data["clip_name"]) if "clip_name" in data else Path(npz_path).stem

    T = lm_seq.shape[0]
    if T < 5: return None

    skill_type = "loco"
    if any(w in clip_name.lower() for w in ["combat", "shoot", "weapon"]): skill_type = "combat"
    elif any(w in clip_name.lower() for w in ["dance", "ballet", "pop"]): skill_type = "dance"
    elif any(w in clip_name.lower() for w in ["precision", "tool"]): skill_type = "precision"

    joint_angles = np.zeros((T, 39), dtype=np.float32)
    root_positions = np.zeros((T, 3), dtype=np.float32)
    root_orientations = np.zeros((T, 3, 3), dtype=np.float32)
    
    left_ankle_z_local = np.zeros(T)
    right_ankle_z_local = np.zeros(T)

    for t in range(T):
        lm = lm_seq[t]

        pelvis, R_root = compute_root_pose(lm)
        root_positions[t] = pelvis
        root_orientations[t] = R_root
        R_inv = R_root.T

        # Extract Local Z-Height
        left_ankle_z_local[t] = (R_inv @ (lm[MP_L_ANK] - pelvis))[2]
        right_ankle_z_local[t] = (R_inv @ (lm[MP_R_ANK] - pelvis))[2]

        # Full Plane-Based Limbs + Extremities
        l_hp, l_hr, l_hy, l_kn, l_ap, l_ar = extract_full_limb_kinematics(lm[MP_L_HIP], lm[MP_L_KNEE], lm[MP_L_ANK], lm[MP_L_FOOT], R_inv, True)
        r_hp, r_hr, r_hy, r_kn, r_ap, r_ar = extract_full_limb_kinematics(lm[MP_R_HIP], lm[MP_R_KNEE], lm[MP_R_ANK], lm[MP_R_FOOT], R_inv, False)
        
        l_sp, l_sr, l_sy, l_el, l_wy, _ = extract_full_limb_kinematics(lm[MP_L_SHLD], lm[MP_L_ELB], lm[MP_L_WRI], lm[MP_L_INDEX], R_inv, True)
        r_sp, r_sr, r_sy, r_el, r_wy, _ = extract_full_limb_kinematics(lm[MP_R_SHLD], lm[MP_R_ELB], lm[MP_R_WRI], lm[MP_R_INDEX], R_inv, False)

        hands = WEAPON_GRIP if skill_type == "combat" else NEUTRAL_HAND

        joint_angles[t] = np.array([
            l_hp, l_hr, l_hy, l_kn, l_ap, l_ar, 0.0, # Left Leg
            r_hp, r_hr, r_hy, r_kn, r_ap, r_ar, 0.0, # Right Leg
            0.0, 0.0, 0.0, # Spine
            l_sp, l_sr, l_sy, l_el, l_wy,            # Left Arm
            r_sp, r_sr, r_sy, r_el, r_wy,            # Right Arm
            *hands
        ])

    # Continuous Unwrapping & Clamping
    unwrapped_angles = np.unwrap(joint_angles, axis=0)
    clamped_angles = np.clip(unwrapped_angles, LOWER_LIMITS, UPPER_LIMITS)

    window = min(15, T - 1 if T % 2 == 0 else T - 2)
    if window > 3:
        smoothed_angles = apply_padded_savgol(clamped_angles, window, 3)
        smoothed_root = apply_padded_savgol(root_positions, window, 3)
        sm_l_ank_z = apply_padded_savgol(left_ankle_z_local.reshape(-1, 1), window, 3).flatten()
        sm_r_ank_z = apply_padded_savgol(right_ankle_z_local.reshape(-1, 1), window, 3).flatten()
    else:
        smoothed_angles, smoothed_root = clamped_angles, root_positions
        sm_l_ank_z, sm_r_ank_z = left_ankle_z_local, right_ankle_z_local

    dt = 1.0 / fps
    joint_velocities = np.gradient(smoothed_angles, dt, axis=0)
    root_linear_vel = np.gradient(smoothed_root, dt, axis=0)

    rotations = R.from_matrix(root_orientations).as_rotvec()
    if window > 3:
        smoothed_rotations = apply_padded_savgol(np.unwrap(rotations, axis=0), window, 3)
    else:
        smoothed_rotations = np.unwrap(rotations, axis=0)
    root_angular_vel = np.gradient(smoothed_rotations, dt, axis=0)

    # Origin Normalization
    normalized_root = np.copy(smoothed_root)
    normalized_root[:, 0] -= smoothed_root[0, 0] 
    normalized_root[:, 1] -= smoothed_root[0, 1] 
    normalized_root[:, 2] = (normalized_root[:, 2] - np.min(normalized_root[:, 2])) + ROBOT_LEG_LENGTH

    # Contact Extraction
    l_vel_z = np.gradient(sm_l_ank_z, dt)
    r_vel_z = np.gradient(sm_r_ank_z, dt)
    min_l, min_r = np.min(sm_l_ank_z), np.min(sm_r_ank_z)
    
    contact_schedule = np.zeros((T, 4), dtype=int)
    for t in range(T):
        l_contact = 1 if (abs(l_vel_z[t]) < 0.2 and abs(sm_l_ank_z[t] - min_l) < 0.05) else 0
        r_contact = 1 if (abs(r_vel_z[t]) < 0.2 and abs(sm_r_ank_z[t] - min_r) < 0.05) else 0
        contact_schedule[t] = [l_contact, r_contact, 0, 0]

    frames = []
    root_quats = R.from_matrix(root_orientations).as_quat()
    
    for t in range(T):
        frames.append({
            "root_pos": normalized_root[t].tolist(),     
            "root_rot": root_quats[t].tolist(),          
            "joint_pos": smoothed_angles[t].tolist(),
            "joint_vel": joint_velocities[t].tolist(),
            "base_vel": root_linear_vel[t].tolist(),
            "base_ang_vel": root_angular_vel[t].tolist(),
            "contact_schedule": contact_schedule[t].tolist(),
            "phase": float(t) / max(1, T - 1), 
            "stiffness_mult": 1.0,
            "rhythm_beat": 0.0
        })

    return {
        "skill_id": clip_name,
        "skill_type": skill_type,
        "duration_s": float(T / fps),
        "frames": frames
    }

def process_file(npz_path: Path, output_dir: Path):
    try:
        clip_data = retarget_clip(str(npz_path))
        if clip_data is None: return f"  ⏭️ Skipped {npz_path.name} (T < 5)"
        out_path = output_dir / (npz_path.stem + ".json")
        with open(out_path, 'w') as f: json.dump(clip_data, f, separators=(',', ':'))
        return f"  ✅ Saved physics frame -> {out_path.name}"
    except Exception as e:
        return f"  ❌ Error on {npz_path.name}: {e}"

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_dir", type=str, default=KINEMATICS_DIR)
    parser.add_argument("--output_dir", type=str, default=RETARGETED_DIR)
    args = parser.parse_args()

    npz_files = list(Path(args.input_dir).glob("*.npz"))
    if not npz_files: return print(f"❌ No .npz files found in {args.input_dir}")

    print(f"🦾 Retargeter initialized – processing {len(npz_files)} clips...")
    with ProcessPoolExecutor(max_workers=MAX_WORKERS) as executor:
        futures = {executor.submit(process_file, p, Path(args.output_dir)): p for p in npz_files}
        for future in as_completed(futures): print(future.result())

if __name__ == "__main__":
    main()