# pipeline/kinematic_retargeter.py
import os
import yaml
import numpy as np
import torch
import smplx
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

# Note: DampedIKSolver is assumed to be in your project utils/
from utils.ik_solver import DampedIKSolver 

class KinematicRetargeter:
    def __init__(self, config_path="cfg/env_config.yaml", urdf_path="assets/usd/revexbot.urdf"):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        self.output_dir = self.config['cluster_paths']['processed_kinematics']
        os.makedirs(self.output_dir, exist_ok=True)
        
        print("[RETARGETER] Loading SMPL-X Biological Prior...")
        self.smplx_layer = smplx.create(
            model_path="models/smplx/SMPLX_NEUTRAL.npz", 
            model_type='smplx', 
            use_pca=False,
            num_betas=10, 
            batch_size=64
        ).cuda()
        self.smplx_layer.eval()
        
        self.ik_solver = DampedIKSolver(urdf_path)
        
        # 🚨 MOONWALK FIX: Leg-Length Ratio Scaling
        # SMPL-X default leg length (hip to ankle) ≈ 0.485m
        self.human_leg_length = 0.485
        self.robot_leg_length = 0.595
        self.robot_arm_length = 0.402
        self.scale_ratio = self.robot_leg_length / self.human_leg_length
        
        # 🚨 42-DoF INDEX MAPPING: Strict URDF ordering for AMP compatibility
        self.urdf_joint_order = [
            # Legs (14)
            'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 
            'left_ankle_pitch_joint', 'left_ankle_roll_joint', 'left_toe_joint',
            'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint', 
            'right_ankle_pitch_joint', 'right_ankle_roll_joint', 'right_toe_joint',
            # Torso & Neck (3)
            'waist_yaw_joint', 'neck_yaw_joint', 'neck_pitch_joint',
            # Arms (10)
            'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 
            'left_elbow_joint', 'left_wrist_yaw_joint',
            'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 
            'right_elbow_joint', 'right_wrist_yaw_joint'
        ]
        # Note: Fingers are handled separately (16 LH + 16 RH = 32). Total DoF = 27 + 32 = 59

    def _axis_convert_yup_to_zup(self, pos_yup):
        return np.stack([pos_yup[:, 0], pos_yup[:, 2], -pos_yup[:, 1]], axis=1)

    def _unwrap_quaternions(self, quats_wxyz):
        unwrapped = np.copy(quats_wxyz)
        for i in range(1, len(unwrapped)):
            dot = np.dot(unwrapped[i], unwrapped[i-1])
            if dot < 0: unwrapped[i] = -unwrapped[i]
        return unwrapped

    def _apply_moe_postural_prior(self, lh_curls: np.ndarray, rh_curls: np.ndarray, weapon_info: dict = None) -> tuple:
        """
        🚨 POSTURAL BRIDGE & WEAPON GRIP OVERRIDE
        Overwrites noisy SMPL-X finger data with MoE-compatible baselines.
        """
        if weapon_info and weapon_info.get('hand') == 'right':
            # Right Hand: Rigid cylindrical grasp around weapon COM
            rh_curls[:, 0:4] = 0.75   # Thumb opposition & base joints
            rh_curls[:, 4:12] = 0.85  # Proximal/Middle wrap
            rh_curls[:, 12:] = 0.65   # Distal lock
            lh_curls[:] = 0.35        # Left hand: Runner's blade
            print("[RETARGETER] Applied Right-Hand Weapon Grip Override.")
        elif weapon_info and weapon_info.get('hand') == 'left':
            lh_curls[:, 0:4] = 0.75
            lh_curls[:, 4:12] = 0.85
            lh_curls[:, 12:] = 0.65
            rh_curls[:] = 0.35
            print("[RETARGETER] Applied Left-Hand Weapon Grip Override.")
        else:
            # Empty Hand: Uniform "Runner's Blade" Prior
            lh_curls[:] = 0.35
            rh_curls[:] = 0.35
            
        return lh_curls, rh_curls

    @torch.no_grad()
    def process_kinematics_batched(self, smplx_params: dict, video_name: str, video_fps: float = 30.0, weapon_info: dict = None) -> str:
        print(f"\n[RETARGETER] Processing {video_name} (FPS: {video_fps})...")
        
        total_frames = smplx_params['body_pose'].shape[0]
        batch_size = 64
        
        all_q_trajectories = []
        all_root_pos_zup = []
        all_root_quat_wxyz = []
        
        for i in tqdm(range(0, total_frames, batch_size), desc="Kinematic Extraction"):
            end_idx = min(i + batch_size, total_frames)
            curr_batch = end_idx - i
            
            # --- 1. Batched SMPL-X Forward Pass ---
            body_pose = torch.tensor(smplx_params['body_pose'][i:end_idx]).cuda()
            global_orient = torch.tensor(smplx_params['global_orient'][i:end_idx]).cuda()
            left_hand = torch.tensor(smplx_params['left_hand_pose'][i:end_idx]).cuda()
            right_hand = torch.tensor(smplx_params['right_hand_pose'][i:end_idx]).cuda()
            
            if 'transl' in smplx_params:
                transl = torch.tensor(smplx_params['transl'][i:end_idx]).cuda()
            else:
                transl = torch.zeros((curr_batch, 3)).cuda()
            
            output = self.smplx_layer(
                transl=transl, 
                global_orient=global_orient,
                body_pose=body_pose,
                left_hand_pose=left_hand,
                right_hand_pose=right_hand
            )
            
            # Root Extraction & Z-Up Flip
            root_pos_yup = output.joints[:, 0, :].cpu().numpy()
            root_rot_mat = output.global_orient[:, 0, :, :].cpu().numpy()
            root_pos_zup = self._axis_convert_yup_to_zup(root_pos_yup)
            
            # 🚨 MOONWALK FIX: Scale root trajectory to match robot leg length
            root_pos_zup *= self.scale_ratio
            
            root_quat_xyzw = R.from_matrix(root_rot_mat).as_quat()
            root_quat_wxyz = np.concatenate([root_quat_xyzw[:, 3:4], root_quat_xyzw[:, :3]], axis=1)
            
            all_root_pos_zup.append(root_pos_zup)
            all_root_quat_wxyz.append(root_quat_wxyz)
            
            # Extract Decoupled Fingers (FK)
            # SMPL-X hand pose is (batch, 15, 3). We map to our 16-DoF URDF fingers.
            lh_raw = left_hand.reshape(curr_batch, 15, 3).cpu().numpy()
            rh_raw = right_hand.reshape(curr_batch, 15, 3).cpu().numpy()
            
            # Map 15 SMPL params to 16 URDF finger joints
            lh_curls = np.concatenate([np.full((curr_batch, 1), 0.5), lh_raw[:, :, 2]], axis=1)
            rh_curls = np.concatenate([np.full((curr_batch, 1), 0.5), rh_raw[:, :, 2]], axis=1)
            lh_curls = np.clip(lh_curls, 0.0, 1.5)
            rh_curls = np.clip(rh_curls, 0.0, 1.5)
            
            # 🚨 POSTURAL BRIDGE: Enforce MoE baselines or weapon grip
            lh_curls, rh_curls = self._apply_moe_postural_prior(lh_curls, rh_curls, weapon_info)
            
            # --- 2. Decoupled Core IK Solve ---
            for j in range(curr_batch):
                joints_yup = output.joints[j:j+1].cpu().numpy()[0]
                smplx_rot_mats = output.global_orient[j].cpu().numpy() 
                
                targets = self._map_core_joints_to_urdf(joints_yup, smplx_rot_mats)
                
                q_core_dict, success = self.ik_solver.solve(targets=targets)
                if not success: 
                    q_core_dict = self.ik_solver.get_default_pose_dict()
                
                # 🚨 42-DoF INDEX MAPPING: Strict ordering
                q_core_array = np.array([q_core_dict.get(name, 0.0) for name in self.urdf_joint_order])
                q_full = np.concatenate([q_core_array, lh_curls[j], rh_curls[j]])
                all_q_trajectories.append(q_full)

        # Build raw arrays
        raw_q = np.array(all_q_trajectories)
        raw_root_pos = np.concatenate(all_root_pos_zup, axis=0)
        raw_root_quat = np.concatenate(all_root_quat_wxyz, axis=0)
        raw_root_quat = self._unwrap_quaternions(raw_root_quat)
        
        # --- 4. Continuous Filtering & Derivation (BEFORE ORIGIN RESET) ---
        # We must calculate velocity on the TRUE global coordinates to prevent
        # artificial momentum spikes at the start of sliced clips.
        pad_len = 15
        padded_q = np.pad(raw_q, ((pad_len, pad_len), (0, 0)), mode='reflect')
        padded_root_pos = np.pad(raw_root_pos, ((pad_len, pad_len), (0, 0)), mode='reflect')
        
        smoothed_padded_q = savgol_filter(padded_q, window_length=15, polyorder=3, axis=0)
        smoothed_padded_root_pos = savgol_filter(padded_root_pos, window_length=15, polyorder=3, axis=0)
        
        dt = 1.0 / video_fps
        padded_v = np.gradient(smoothed_padded_q, dt, axis=0)
        padded_root_v = np.gradient(smoothed_padded_root_pos, dt, axis=0)
        
        smoothed_q = smoothed_padded_q[pad_len:-pad_len]
        smoothed_v = padded_v[pad_len:-pad_len]
        true_root_pos = smoothed_padded_root_pos[pad_len:-pad_len]
        true_root_v = padded_root_v[pad_len:-pad_len]
        
        # --- 5. Origin Normalization (AFTER Derivation) ---
        # Now we can safely zero the origin for the position tensor,
        # knowing the velocity tensor holds the true momentum.
        normalized_root_pos = np.copy(true_root_pos)
        normalized_root_pos[:, 0] -= true_root_pos[0, 0] 
        normalized_root_pos[:, 1] -= true_root_pos[0, 1] 
        normalized_root_pos[:, 2] = (normalized_root_pos[:, 2] - np.min(normalized_root_pos[:, 2])) + self.robot_leg_length
        
        # Final Tensor: [RootPos(3), RootQuat(4), Joints(59), RootVel(3), JointVel(59)]
        final_trajectory = np.concatenate([
            normalized_root_pos, 
            raw_root_quat, 
            smoothed_q, 
            true_root_v, # 🚨 ADDED: Explicit Root Velocity
            smoothed_v
        ], axis=1)
        
        output_path = os.path.join(self.output_dir, video_name.replace(".mp4", ".npy"))
        np.save(output_path, final_trajectory)
        print(f"[RETARGETER] ✅ Saved URDF-synced array {final_trajectory.shape} to {output_path}")
        return output_path

    def _map_core_joints_to_urdf(self, joints_yup, smplx_full_rot_mats):
        joints_zup = self._axis_convert_yup_to_zup(joints_yup.reshape(-1, 3)).reshape(-1, 3)
        pelvis = joints_zup[0]
        
        def scale_limb(start, end, target_len):
            vec = end - start
            norm = np.linalg.norm(vec)
            if norm < 1e-6: return end
            return start + (vec / norm) * target_len
            
        targets = {
            'lh_palm': scale_limb(joints_zup[16], joints_zup[20], self.robot_arm_length) - pelvis,      
            'rh_palm': scale_limb(joints_zup[17], joints_zup[21], self.robot_arm_length) - pelvis, 
            'left_ankle_pitch_link': scale_limb(pelvis, joints_zup[7], self.robot_leg_length) - pelvis,  
            'right_ankle_pitch_link': scale_limb(pelvis, joints_zup[8], self.robot_leg_length) - pelvis  
        }
        
        lh_human_rot = smplx_full_rot_mats[20]
        rh_human_rot = smplx_full_rot_mats[21]
        
        rh_correction = R.from_euler('xyz', [-0.7854, -3.14159, 1.5708]).as_matrix()
        lh_correction = R.from_euler('xyz', [0.7854, -3.14159, -1.5708]).as_matrix()
        
        targets['rh_palm_rot'] = np.matmul(rh_human_rot, rh_correction)  
        targets['lh_palm_rot'] = np.matmul(lh_human_rot, lh_correction)

        return targets

if __name__ == "__main__":
    retargeter = KinematicRetargeter()