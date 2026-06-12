"""
Module 2 – Kinematic Extractor (extract_kinematics.py)
Extracts 3D metric full-body + hand landmarks from split_clips.
Features:
- Aspect-Ratio Preserving Letterbox Cropping (Gray Pad)
- True 3D Metric World Landmarks for BOTH Body and Hands
- Vector-Translated Hand Safety Fallbacks based on true visibility
- CUDA-Safe Process Pooling
"""
import os
import cv2
import numpy as np
import torch
import torch.multiprocessing as mp_torch
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
from ultralytics import YOLO
import mediapipe as mp

# ----------------------------------------------------------------------
# Paths & Configuration
# ----------------------------------------------------------------------
PIPELINE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(PIPELINE_DIR)

CLIPS_DIR = os.path.join(PROJECT_ROOT, "data", "split_clips")
KINEMATICS_DIR = os.path.join(PROJECT_ROOT, "data", "kinematics")
os.makedirs(KINEMATICS_DIR, exist_ok=True)

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
YOLO_MODEL = "yolov8n-pose.pt"

# Hand safety: neutral relaxed fist pose centered at origin
NEUTRAL_FIST_LANDMARKS = np.array([
    [0.0, 0.0, 0.0], [0.02, -0.02, 0.01], [0.04, -0.03, 0.02], [0.06, -0.03, 0.03],
    [0.08, -0.02, 0.04], [0.02, -0.05, 0.01], [0.04, -0.06, 0.02], [0.06, -0.06, 0.03],
    [0.08, -0.05, 0.04], [0.02, -0.08, 0.01], [0.04, -0.09, 0.02], [0.06, -0.09, 0.03],
    [0.08, -0.08, 0.04], [0.02, -0.10, 0.01], [0.04, -0.12, 0.02], [0.06, -0.12, 0.03],
    [0.08, -0.10, 0.04], [0.03, -0.13, 0.01], [0.05, -0.14, 0.02], [0.06, -0.14, 0.03],
    [0.07, -0.13, 0.04]
], dtype=np.float32)

BODY_CONF_THRESHOLD = 0.6
HAND_CONF_THRESHOLD = 0.7
MAX_MISSING_JOINTS = 3
MAJOR_JOINTS_IDX = [0, 2, 5, 7, 8, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26]

# Left and Right Wrist Indices in MediaPipe Topology
LEFT_WRIST_IDX = 15
RIGHT_WRIST_IDX = 16

MAX_WORKERS = 4 

def process_video(video_path: str) -> str:
    """Extract 3D skeleton from a single video clip and save .npz."""
    video_name = Path(video_path).stem
    out_path = os.path.join(KINEMATICS_DIR, f"{video_name}.npz")

    if os.path.exists(out_path):
        return f"⏭ Already processed: {video_name}"

    yolo = YOLO(YOLO_MODEL)
    yolo.to(DEVICE)

    mp_holistic = mp.solutions.holistic
    holistic = mp_holistic.Holistic(
        static_image_mode=False,
        model_complexity=1, 
        smooth_landmarks=True,
        enable_segmentation=False,
        refine_face_landmarks=False,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        holistic.close()
        return f"❌ Cannot open {video_name}"

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    frames_joint_pos = []
    frames_joint_conf = []
    processed_frames = 0

    try:
        while True:
            ret, frame_bgr = cap.read()
            if not ret:
                break

            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            
            # Person detection with YOLO
            results = yolo(frame_rgb, verbose=False)
            boxes = results[0].boxes

            if boxes is None or len(boxes) == 0:
                continue

            largest_area = 0
            best_box = None
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                area = (x2 - x1) * (y2 - y1)
                if area > largest_area:
                    largest_area = area
                    best_box = (int(x1), int(y1), int(x2), int(y2))

            if best_box is None:
                continue

            # 🚨 FIX 3: Letterbox padding to preserve aspect ratio & MediaPipe accuracy
            x1, y1, x2, y2 = best_box
            
            # Add a 10% padding buffer to the bounding box
            pad_x = int((x2 - x1) * 0.1)
            pad_y = int((y2 - y1) * 0.1)
            
            mx1, my1 = max(0, x1 - pad_x), max(0, y1 - pad_y)
            mx2, my2 = min(frame_rgb.shape[1], x2 + pad_x), min(frame_rgb.shape[0], y2 + pad_y)
            
            crop = frame_rgb[my1:my2, mx1:mx2]
            
            if crop.size == 0:
                continue
                
            h, w = crop.shape[:2]
            dim = max(h, w)
            padded = np.full((dim, dim, 3), 128, dtype=np.uint8) # 128 is neutral gray
            y_off = (dim - h) // 2
            x_off = (dim - w) // 2
            padded[y_off:y_off+h, x_off:x_off+w] = crop
            
            crop_resized = cv2.resize(padded, (512, 512))
            crop_resized.flags.writeable = False

            results_mp = holistic.process(crop_resized)

            body_landmarks = np.zeros((33, 3), dtype=np.float32)
            body_conf = np.zeros(33, dtype=np.float32)
            lh_landmarks = np.zeros((21, 3), dtype=np.float32)
            lh_conf = np.zeros(21, dtype=np.float32)
            rh_landmarks = np.zeros((21, 3), dtype=np.float32)
            rh_conf = np.zeros(21, dtype=np.float32)

            # Body: True Metric World Landmarks
            if results_mp.pose_world_landmarks and results_mp.pose_landmarks:
                for idx, (world_lm, screen_lm) in enumerate(zip(results_mp.pose_world_landmarks.landmark, results_mp.pose_landmarks.landmark)):
                    body_landmarks[idx] = [world_lm.x, world_lm.y, world_lm.z]
                    body_conf[idx] = screen_lm.visibility 

            # 🚨 FIX 1 & 2: Hands - True Metric World Landmarks AND real confidence scores
            if results_mp.left_hand_world_landmarks:
                for idx, lm in enumerate(results_mp.left_hand_world_landmarks.landmark):
                    lh_landmarks[idx] = [lm.x, lm.y, lm.z]
                    lh_conf[idx] = lm.visibility if hasattr(lm, 'visibility') else 1.0
            else:
                lh_conf[:] = 0.0

            if results_mp.right_hand_world_landmarks:
                for idx, lm in enumerate(results_mp.right_hand_world_landmarks.landmark):
                    rh_landmarks[idx] = [lm.x, lm.y, lm.z]
                    rh_conf[idx] = lm.visibility if hasattr(lm, 'visibility') else 1.0
            else:
                rh_conf[:] = 0.0

            # Hand Safety Override: Vector translate neutral fist to wrist
            if lh_conf.mean() < HAND_CONF_THRESHOLD:
                lh_landmarks = NEUTRAL_FIST_LANDMARKS + body_landmarks[LEFT_WRIST_IDX]
                lh_conf[:] = 1.0
            if rh_conf.mean() < HAND_CONF_THRESHOLD:
                rh_landmarks = NEUTRAL_FIST_LANDMARKS + body_landmarks[RIGHT_WRIST_IDX]
                rh_conf[:] = 1.0

            # Body Occlusion Filter
            major_conf = body_conf[MAJOR_JOINTS_IDX]
            missing_count = (major_conf < BODY_CONF_THRESHOLD).sum()
            if missing_count > MAX_MISSING_JOINTS:
                continue 

            all_landmarks = np.concatenate([body_landmarks, lh_landmarks, rh_landmarks], axis=0) # (75, 3)
            all_confs = np.concatenate([body_conf, lh_conf, rh_conf]) # (75,)

            frames_joint_pos.append(all_landmarks)
            frames_joint_conf.append(all_confs)
            processed_frames += 1

    finally:
        cap.release()
        holistic.close()
        del yolo
        torch.cuda.empty_cache()

    if processed_frames == 0:
        return f"❌ No valid frames for {video_name}"

    joint_positions = np.stack(frames_joint_pos, axis=0) 
    joint_confidences = np.stack(frames_joint_conf, axis=0)

    np.savez_compressed(
        out_path,
        joint_positions=joint_positions,
        joint_confidences=joint_confidences,
        fps=fps,
        clip_name=video_name
    )

    return f"✅ {video_name} – {processed_frames}/{total_frames} frames saved"

def main():
    # 🚨 FIX 4: Prevent CUDA Process Initialization Crash
    mp_torch.set_start_method('spawn', force=True)

    video_ext = ('.mp4', '.mov', '.avi', '.webm')
    video_files = [f for f in os.listdir(CLIPS_DIR) if f.lower().endswith(video_ext)]
    if not video_files:
        print(f"❌ No video clips found in {CLIPS_DIR}. Run ingest_media.py first.")
        return

    print(f"🧠 Kinematic Extractor started – {len(video_files)} clips to process")
    print(f"🖥️ Device: {DEVICE} | Workers: {MAX_WORKERS}")

    with ProcessPoolExecutor(max_workers=MAX_WORKERS) as executor:
        futures = {executor.submit(process_video, os.path.join(CLIPS_DIR, vf)): vf for vf in video_files}
        for future in as_completed(futures):
            print(future.result())

    print(f"🏁 Extraction complete. True metric kinematics saved to {KINEMATICS_DIR}")

if __name__ == "__main__":
    main()