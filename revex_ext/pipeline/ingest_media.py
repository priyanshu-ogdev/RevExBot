"""
Module 1 – Scene Splitter (Motion-Based)
Segments long videos into single-skill clips based on motion pauses.
Uses downscaled Farneback optical flow for CPU survival.
Uses FFmpeg with CRF 18, frame-accurate seeking, and audio stripping.
Reads from root ../data/raw_media/ and writes to ../data/split_clips/
"""

import os
import cv2
import subprocess
import shutil
import numpy as np
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed

# ----------------------------------------------------------------------
# 1. DIRECTORY MAPPING (Root-Level Data Architecture)
# ----------------------------------------------------------------------
# __file__ is in revex_ext/pipeline/
PIPELINE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(PIPELINE_DIR)

# Data goes to root, requirements stay in pipeline
RAW_DIR = os.path.join(PROJECT_ROOT, "data", "raw_media")
CLIPS_DIR = os.path.join(PROJECT_ROOT, "data", "split_clips")
REQ_DIR = os.path.join(PIPELINE_DIR, "req")

os.makedirs(CLIPS_DIR, exist_ok=True)

# ----------------------------------------------------------------------
# 2. CONFIGURATION
# ----------------------------------------------------------------------
MIN_CLIP_DURATION = 1.5          # seconds
MAX_CLIP_DURATION = 8.0          # seconds
MOTION_PAUSE_THRESHOLD = 0.02    # average flow magnitude
PAUSE_HOLD_FRAMES = 15           # consecutive still frames before cutting

# 🚨 FIX 3: Dynamic Thread Limiting (Leaves breathing room for the OS and FFmpeg)
MAX_WORKERS = max(1, os.cpu_count() // 2)

def optical_flow_magnitude(prev_gray, curr_gray):
    """Average Farneback optical flow magnitude between two downscaled frames."""
    flow = cv2.calcOpticalFlowFarneback(prev_gray, curr_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    mag = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)
    return np.mean(mag)

def split_video_motion(video_path: str, output_dir: str, ffmpeg_path: str):
    """Split video into motion-segmented clips and export via FFmpeg."""
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"  ❌ Cannot open {video_path}")
        return

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    clips = []

    try:
        ret, prev_frame = cap.read()
        if not ret:
            return
            
        # Downscale and Blur for CPU survival and noise immunity
        prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
        prev_small = cv2.resize(prev_gray, (256, 144))
        prev_small = cv2.GaussianBlur(prev_small, (5, 5), 0)

        frame_idx = 0
        clip_start = 0
        still_counter = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            frame_idx += 1

            curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            curr_small = cv2.resize(curr_gray, (256, 144))
            curr_small = cv2.GaussianBlur(curr_small, (5, 5), 0)
            
            motion = optical_flow_magnitude(prev_small, curr_small)
            prev_small = curr_small

            if motion < MOTION_PAUSE_THRESHOLD:
                still_counter += 1
            else:
                still_counter = 0

            clip_duration = (frame_idx - clip_start) / fps
            
            if still_counter >= PAUSE_HOLD_FRAMES and clip_duration >= MIN_CLIP_DURATION:
                clips.append((clip_start / fps, (frame_idx - still_counter) / fps))
                clip_start = frame_idx
                still_counter = 0
            elif clip_duration > MAX_CLIP_DURATION:
                clips.append((clip_start / fps, frame_idx / fps))
                clip_start = frame_idx
                still_counter = 0

        final_dur = (total_frames - clip_start) / fps
        if final_dur >= MIN_CLIP_DURATION:
            clips.append((clip_start / fps, total_frames / fps))

    finally:
        cap.release()

    # --- FFmpeg Export Logic ---
    video_name = Path(video_path).stem

    for i, (start_sec, end_sec) in enumerate(clips):
        duration = end_sec - start_sec
        if duration < MIN_CLIP_DURATION:
            continue
            
        out_path = os.path.join(output_dir, f"{video_name}_clip{i:03d}.mp4")
        
        # 🚨 FIX 1 & 3: -ss AFTER -i for frame-accuracy. Threads reduced to 2.
        cmd = [
            ffmpeg_path, "-y", "-hide_banner", "-loglevel", "error",
            "-i", video_path, "-ss", str(start_sec), "-t", str(duration),
            "-c:v", "libx264", "-preset", "fast", "-crf", "18",
            "-threads", "2", "-an",
            out_path
        ]
        try:
            subprocess.run(cmd, check=True)
            print(f"    -> {Path(out_path).name} ({duration:.1f}s)")
        except subprocess.CalledProcessError as e:
            print(f"    ❌ FFmpeg failed on {Path(out_path).name}: {e}")

    if clips:
        print(f"  ✅ {len(clips)} clips saved from {video_name}.")

def process_video(video_path: str, ffmpeg_path: str):
    try:
        split_video_motion(video_path, CLIPS_DIR, ffmpeg_path)
        return f"✅ Processed: {Path(video_path).name}"
    except Exception as e:
        return f"❌ Failed: {Path(video_path).name} ({e})"

def main():
    video_ext = ('.mp4', '.mov', '.avi', '.webm')
    video_files = [f for f in os.listdir(RAW_DIR) if f.lower().endswith(video_ext)]
    
    if not video_files:
        print(f"❌ No videos found in {RAW_DIR}. Please add raw media first.")
        return

    # 🚨 FIX 2: Windows-Hardened FFmpeg Resolution
    local_ffmpeg = os.path.join(REQ_DIR, "ffmpeg.exe")
    if os.path.exists(local_ffmpeg):
        ffmpeg_path = local_ffmpeg
    else:
        ffmpeg_path = shutil.which("ffmpeg")
        
    if not ffmpeg_path:
        print("❌ FFmpeg not found in req/ folder or system PATH.")
        return

    print(f"🚀 Ingestor Module Initialized. Found {len(video_files)} raw videos.")
    print(f"⚙️  FFmpeg Binary: {ffmpeg_path}")
    print(f"📂 Output Path: {CLIPS_DIR}")
    print(f"🔪 Motion-based splitting on {MAX_WORKERS} threads...\n")
    
    with ThreadPoolExecutor(max_workers=MAX_WORKERS) as executor:
        futures = {executor.submit(process_video, os.path.join(RAW_DIR, vf), ffmpeg_path): vf for vf in video_files}
        for future in as_completed(futures):
            print(future.result())

    print(f"\n🏁 Scene splitting complete. All atomic clips saved to {CLIPS_DIR}")

if __name__ == "__main__":
    main()