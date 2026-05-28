import os
import yaml
import numpy as np
import torch
import smplx
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as R
from utils.ik_solver import DampedIKSolver # Custom solver mapping to RevEx URDF
from tqdm import tqdm

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
        
        self.robot_leg_length = 0.595 
        self.robot_arm_length = 0.402 
        
        self.core_joint_names = [
            'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint', 'left_toe_joint',
            'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint', 'right_toe_joint',
            'waist_yaw_joint',
            'neck_yaw_joint', 'neck_pitch_joint',
            'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_yaw_joint',
            'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_yaw_joint'
        ]

    def _axis_convert_yup_to_zup(self, pos_yup):
        return np.stack([pos_yup[:, 0], pos_yup[:, 2], -pos_yup[:, 1]], axis=1)

    def _unwrap_quaternions(self, quats_wxyz):
        unwrapped = np.copy(quats_wxyz)
        for i in range(1, len(unwrapped)):
            dot = np.dot(unwrapped[i], unwrapped[i-1])
            if dot < 0:
                unwrapped[i] = -unwrapped[i]
        return unwrapped

    def _extract_smplx_finger_angles(self, smplx_hand_pose):
        curls = smplx_hand_pose[:, :, 2].cpu().numpy() 
        curls = np.clip(curls, 0.0, 1.5)
        
        batch_size = curls.shape[0]
        thumb_base = np.full((batch_size, 1), 0.5) 
        
        urdf_curls = np.concatenate([
            thumb_base,    
            curls[:, 0:3], 
            curls[:, 3:15] 
        ], axis=1)
        
        return urdf_curls 

    @torch.no_grad()
    def process_kinematics_batched(self, smplx_params: dict, video_name: str, video_fps: float = 30.0) -> str:
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
            
            # GAP 82 FIX: Inject Global Translation
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
            root_quat_xyzw = R.from_matrix(root_rot_mat).as_quat()
            root_quat_wxyz = np.concatenate([root_quat_xyzw[:, 3:4], root_quat_xyzw[:, :3]], axis=1)
            
            all_root_pos_zup.append(root_pos_zup)
            all_root_quat_wxyz.append(root_quat_wxyz)
            
            # Extract Decoupled Fingers (FK)
            lh_curls = self._extract_smplx_finger_angles(left_hand.reshape(curr_batch, 15, 3))
            rh_curls = self._extract_smplx_finger_angles(right_hand.reshape(curr_batch, 15, 3))
            
            # --- 2. Decoupled Core IK Solve ---
            for j in range(curr_batch):
                joints_yup = output.joints[j:j+1].cpu().numpy()[0]
                smplx_rot_mats = output.global_orient[j].cpu().numpy() 
                
                targets = self._map_core_joints_to_urdf(joints_yup, smplx_rot_mats)
                
                q_core_dict, success = self.ik_solver.solve(targets=targets)
                if not success: 
                    q_core_dict = self.ik_solver.get_default_pose_dict()
                
                # --- 3. Array Splicing (Total: 59 DoF) ---
                q_core_array = np.array([q_core_dict.get(name, 0.0) for name in self.core_joint_names])
                q_full = np.concatenate([q_core_array, lh_curls[j], rh_curls[j]])
                all_q_trajectories.append(q_full)

        # Build raw arrays
        raw_q = np.array(all_q_trajectories)
        raw_root_pos = np.concatenate(all_root_pos_zup, axis=0)
        raw_root_quat = np.concatenate(all_root_quat_wxyz, axis=0)
        raw_root_quat = self._unwrap_quaternions(raw_root_quat)
        
        # --- 4. Origin Normalization (Centering the Trajectory) ---
        # Subtract the MEAN position to keep the robot centered in the arena
        # This prevents the "Treadmill" effect while avoiding infinite drift
        mean_x = np.mean(raw_root_pos[:, 0])
        mean_y = np.mean(raw_root_pos[:, 1])
        
        raw_root_pos[:, 0] -= mean_x 
        raw_root_pos[:, 1] -= mean_y 
        raw_root_pos[:, 2] = (raw_root_pos[:, 2] - np.min(raw_root_pos[:, 2])) + self.robot_leg_length
        
        # --- 5. Continuous Filtering & Derivation ---
        pad_len = 15
        padded_q = np.pad(raw_q, ((pad_len, pad_len), (0, 0)), mode='reflect')
        smoothed_padded_q = savgol_filter(padded_q, window_length=15, polyorder=3, axis=0)
        
        dt = 1.0 / video_fps
        padded_v = np.gradient(smoothed_padded_q, dt, axis=0)
        
        smoothed_q = smoothed_padded_q[pad_len:-pad_len]
        smoothed_v = padded_v[pad_len:-pad_len]
        
        final_trajectory = np.concatenate([
            raw_root_pos, 
            raw_root_quat, 
            smoothed_q, 
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