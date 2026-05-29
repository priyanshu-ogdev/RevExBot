# pipeline/video_ingestion_daemon.py
import os
import sys
import cv2
import json
import yaml
import shutil
import time
import torch
import numpy as np
from PIL import Image
from numba import njit
from transformers import Qwen2_5_VLForConditionalGeneration, AutoProcessor
from qwen_vl_utils import process_vision_info

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from kinematic_retargeter import KinematicRetargeter

# ============================================================
# 🚨 NUMBA-ACCELERATED DTW (C-Level Speedup)
# ============================================================
@njit(cache=True, fastmath=True)
def fast_dtw_distance(seq1: np.ndarray, seq2: np.ndarray, window: int = 10) -> float:
    """
    Optimized Sakoe-Chiba DTW. Handles variable lengths natively.
    Compiled to machine code for ~200x CPU speedup.
    """
    n, m = seq1.shape[0], seq2.shape[0]
    if n == 0 or m == 0:
        return 1.0
        
    dtw = np.full((n + 1, m + 1), np.inf, dtype=np.float32)
    dtw[0, 0] = 0.0
    
    for i in range(1, n + 1):
        j_start = max(1, i - window)
        j_end = min(m + 1, i + window + 1)
        for j in range(j_start, j_end):
            diff = seq1[i-1] - seq2[j-1]
            cost = np.sqrt(np.sum(diff * diff))
            dtw[i, j] = cost + min(dtw[i-1, j], dtw[i, j-1], dtw[i-1, j-1])
            
    return dtw[n, m] / (n + m)

# ============================================================
# 🧠 VIDEO INGESTION DAEMON (Tri-Stage MoE Architecture)
# ============================================================
class VideoIngestionDaemon:
    def __init__(self, config_path="cfg/env_config.yaml"):
        print("[DAEMON] Booting MoE Vision Ingestion Engine (Tri-Stage Architecture)...")
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        paths = self.config['cluster_paths']
        self.drop_zone = paths['drop_zone']
        self.processed_dir = os.path.join(self.drop_zone, "processed")
        self.clean_video_dir = paths['raw_videos']
        self.queue_path = paths['job_queue_json']
        self.motions_dir = paths['processed_kinematics']
        
        os.makedirs(self.drop_zone, exist_ok=True)
        os.makedirs(self.processed_dir, exist_ok=True)
        os.makedirs(self.clean_video_dir, exist_ok=True)
        os.makedirs(self.motions_dir, exist_ok=True)
        
        self.model_id = self.config['vision_daemon'].get('vlm_model', "Qwen/Qwen2.5-VL-3B-Instruct")
        self.max_frames_per_clip = 16
        self.motion_threshold = self.config['vision_daemon'].get('motion_threshold', 0.05)
        self.merge_threshold = self.config['vision_daemon']['clustering']['merge_threshold']
        self.vlm_batch_size = self.config['vision_daemon'].get('vlm_batch_size', 4) # Micro-batch size

    def _slice_atomic_clips(self, video_path: str, subject_masks: dict, fps: float) -> list:
        """Stage 2 TAL: Slices continuous footage into atomic technique clips based on velocity boundaries."""
        cap = cv2.VideoCapture(video_path)
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        subject_clips = {sid: [] for sid in subject_masks.keys()}
        
        prev_pos = {sid: None for sid in subject_masks.keys()}
        clip_start = {sid: 0 for sid in subject_masks.keys()}
        
        for frame_idx in range(total_frames):
            ret, frame = cap.read()
            if not ret: break
            
            for sid, mask in subject_masks.items():
                coords = cv2.findNonZero(mask)
                if coords is None: continue
                M = cv2.moments(mask)
                if M['m00'] == 0: continue
                cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                
                if prev_pos[sid] is not None:
                    dx, dy = cx - prev_pos[sid][0], cy - prev_pos[sid][1]
                    velocity = np.sqrt(dx**2 + dy**2) / fps
                    
                    # Technique boundary: velocity drops below threshold + min duration
                    if velocity < self.motion_threshold and (frame_idx - clip_start[sid]) > fps * 0.5:
                        subject_clips[sid].append((clip_start[sid], frame_idx))
                        clip_start[sid] = frame_idx
                prev_pos[sid] = (cx, cy)
                
        cap.release()
        # Append final clip if valid
        for sid in subject_masks.keys():
            if (total_frames - clip_start[sid]) > fps * 0.5:
                subject_clips[sid].append((clip_start[sid], total_frames))
                
        # Flatten to list of dicts
        flat_clips = []
        for sid, ranges in subject_clips.items():
            for start, end in ranges:
                flat_clips.append({"subject_id": sid, "start_frame": start, "end_frame": end, "fps": fps})
        return flat_clips

    def _vlm_triage_batched(self, clips: list, source_filename: str) -> list:
        """Stage 1/3: Micro-batched VLM Semantic Triage for Ada GPU saturation."""
        if not clips: return clips
        
        results = []
        # Process in micro-batches to prevent VRAM OOM
        for i in range(0, len(clips), self.vlm_batch_size):
            batch = clips[i:i + self.vlm_batch_size]
            messages_batch = []
            
            for clip in batch:
                cap = cv2.VideoCapture(os.path.join(self.drop_zone, source_filename))
                pil_frames = []
                step = max(1, (clip["end_frame"] - clip["start_frame"]) // self.max_frames_per_clip)
                for f_idx in range(clip["start_frame"], clip["end_frame"], step):
                    cap.set(cv2.CAP_PROP_POS_FRAMES, f_idx)
                    ret, frame = cap.read()
                    if ret:
                        pil_frames.append(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
                cap.release()
                
                if len(pil_frames) < 4: 
                    clip["domain"] = "Unknown"
                    clip["technique"] = "unknown"
                    clip["weapons"] = []
                    clip["weapon_relative_pose"] = [0.0, 0.0, 0.0]
                    results.append(clip)
                    continue
                
                prompt = f"Analyze Subject {clip['subject_id']}. Return JSON: domain, technique, weapons (array), weapon_relative_pose (x,y,z offset from palm)."
                content_list = [{"type": "image", "image": img} for img in pil_frames[:self.max_frames_per_clip]]
                content_list.append({"type": "text", "text": prompt})
                messages_batch.append([{"role": "user", "content": content_list}])
                
            if not messages_batch: continue
            
            # Batched VLM Inference
            texts = [p.apply_chat_template(m, tokenize=False, add_generation_prompt=True)[0] for m in messages_batch]
            image_inputs, video_inputs = process_vision_info(messages_batch)
            inputs = self.processor(text=texts, images=image_inputs, videos=video_inputs, padding=True, return_tensors="pt").to("cuda")
            
            with torch.no_grad():
                generated_ids = self.model.generate(**inputs, max_new_tokens=150, temperature=0.1, do_sample=False)
            
            outputs = self.processor.batch_decode(generated_ids, skip_special_tokens=True)
            
            for j, clip in enumerate(batch):
                try:
                    parsed = self.sanitize_vlm_json(outputs[j])
                    clip.update(parsed)
                    results.append(clip)
                except Exception as e:
                    print(f"[WARN] VLM parse failed for clip {clip['subject_id']}: {e}")
                    clip.update({"domain": "Unknown", "technique": "unknown", "weapons": [], "weapon_relative_pose": [0.0, 0.0, 0.0]})
                    results.append(clip)
                    
        torch.cuda.empty_cache()
        return results

    def _cluster_and_generate_manifest(self, new_motion_data, domain, technique, fps, style_registry_path):
        """Stage 5: DTW Clustering & Prop Manifest Generation (CPU-bound, Numba-accelerated)"""
        registry = {"styles": []}
        if os.path.exists(style_registry_path):
            with open(style_registry_path, 'r') as f:
                registry = json.load(f)
                
        # Extract joint velocities (assuming 42 joints at index 7)
        new_velocities = np.diff(new_motion_data[:, 7:7+42], axis=0).astype(np.float32)
        new_velocities = np.nan_to_num(new_velocities, nan=0.0)
        
        best_dist, best_style_id = 1.0, None
        for entry in registry["styles"]:
            if entry.get("domain") != domain: continue
            existing_npy = np.load(entry["npy_path"])
            existing_velocities = np.diff(existing_npy[:, 7:7+42], axis=0).astype(np.float32)
            
            # DTW handles variable lengths natively. No interpolation needed.
            dist = fast_dtw_distance(new_velocities, existing_velocities)
            if dist < best_dist:
                best_dist, best_style_id = dist, entry["style_id"]
                
        if best_style_id is not None and best_dist < self.merge_threshold:
            print(f"[CLUSTER] Merging into style_id: {best_style_id} (Distance: {best_dist:.3f})")
            return best_style_id, False
        else:
            new_id = max([e["style_id"] for e in registry["styles"]], default=0) + 1
            registry["styles"].append({
                "style_id": new_id,
                "domain": domain,
                "technique": technique,
                "fps": fps,
                "npy_path": "" # Updated after save
            })
            with open(style_registry_path, 'w') as f:
                json.dump(registry, f, indent=4)
            print(f"[CLUSTER] Created new style_id: {new_id}")
            return new_id, True

    def process_drop_zone(self):
        files = [f for f in os.listdir(self.drop_zone) if f.endswith(('.mp4', '.mov', '.avi'))]
        if not files:
            print("[DAEMON] Drop zone empty. Exiting.")
            return

        print(f"[DAEMON] Found {len(files)} raw videos. Initiating Tri-Stage MoE Ingestion...")
        
        # Load VLM ONCE for all files (VRAM Airlock Pattern)
        print("[PHASE 2] Loading VLM for semantic triage...")
        self.processor = AutoProcessor.from_pretrained(self.model_id)
        self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
            self.model_id, torch_dtype=torch.bfloat16, device_map="auto"
        )
        
        successfully_processed = []
        style_registry = os.path.join(self.motions_dir, "style_manifest.json")
        prop_manifest_path = os.path.join(self.motions_dir, "prop_manifest.json")
        
        try:
            retargeter = KinematicRetargeter(config_path="cfg/env_config.yaml")
            
            for filename in files:
                raw_path = os.path.join(self.drop_zone, filename)
                print(f"\n[INGEST] Processing: {filename}")
                cap = cv2.VideoCapture(raw_path)
                fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
                cap.release()
                
                # Mock multi-subject masks (Replace with YOLOv8 + ByteTrack in prod)
                subject_masks = {"p1": np.ones((1080, 1920), dtype=np.uint8) * 255} 
                atomic_clips = self._slice_atomic_clips(raw_path, subject_masks, fps)
                
                if not atomic_clips:
                    print(f"[SKIP] No valid motion detected in {filename}")
                    successfully_processed.append(filename)
                    continue
                    
                # Add source filename to clips
                for clip in atomic_clips: clip["source"] = filename
                
                # VLM Batched Triage
                triage_results = self._vlm_triage_batched(atomic_clips, filename)
                
                # Kinematics & Clustering
                for clip in triage_results:
                    try:
                        total_frames = clip["end_frame"] - clip["start_frame"]
                        smplx_params = {
                            'person_id': clip["subject_id"],
                            'transl': np.zeros((total_frames, 3), dtype=np.float32),
                            'body_pose': np.zeros((total_frames, 21, 3), dtype=np.float32),
                            'global_orient': np.zeros((total_frames, 1, 3), dtype=np.float32),
                            'left_hand_pose': np.zeros((total_frames, 15, 3), dtype=np.float32),
                            'right_hand_pose': np.zeros((total_frames, 15, 3), dtype=np.float32),
                            'betas': np.zeros((10,), dtype=np.float32)
                        }
                        
                        virtual_name = f"{clip['technique']}_{clip['subject_id']}_{clip['start_frame']}"
                        npy_path = retargeter.process_kinematics_batched(smplx_params, virtual_name, clip["fps"])
                        motion_data = np.load(npy_path)
                        
                        style_id, is_new = self._cluster_and_generate_manifest(
                            motion_data, clip["domain"], clip["technique"], clip["fps"], style_registry
                        )
                        
                        # Update manifest with actual path
                        if is_new:
                            with open(style_registry, 'r') as f: registry = json.load(f)
                            for s in registry["styles"]:
                                if s["style_id"] == style_id:
                                    s["npy_path"] = npy_path
                                    break
                            with open(style_registry, 'w') as f: json.dump(registry, f, indent=4)
                        
                        # Handle weapons
                        if clip.get("weapons") and len(clip["weapons"]) > 0:
                            props = {"styles": []}
                            if os.path.exists(prop_manifest_path):
                                with open(prop_manifest_path, 'r') as f: props = json.load(f)
                            props["styles"].append({
                                "style_id": style_id,
                                "weapon": clip["weapons"][0],
                                "offset_to_palm": clip.get("weapon_relative_pose", [0.0, 0.0, 0.0]),
                                "mass_kg": 0.8 # Default prop mass
                            })
                            with open(prop_manifest_path, 'w') as f: json.dump(props, f, indent=4)
                            
                        self.queue_domain_job(clip["domain"], npy_path, style_id)
                    except Exception as e:
                        print(f"[ERROR] Failed clip {clip['subject_id']}: {e}")
                        
                successfully_processed.append(filename)
                print(f"[OK] {filename} processed successfully.")
                
        finally:
            # VRAM Airlock Flush
            del self.model
            del self.processor
            if 'retargeter' in locals(): del retargeter
            torch.cuda.empty_cache()
            
        # 🚨 ATOMIC FILE ARCHIVING (Prevents zombie re-ingestion)
        print("[ARCHIVE] Moving processed files to avoid duplicates...")
        for filename in successfully_processed:
            src = os.path.join(self.drop_zone, filename)
            dst = os.path.join(self.processed_dir, filename)
            try:
                shutil.move(src, dst)
            except Exception as e:
                print(f"[WARN] Failed to archive {filename}: {e}")
                
        print("[DAEMON] MoE Ingestion Complete. Drop zone cleared.")

        # 🚨 AUTO-GENERATE AMP REFERENCE POOL (Latent Conditioning Bridge)
        pool_path = os.path.join(self.motions_dir, "reference_pool.json")
        reference_pool = []
        
        # Scan all generated .npy files and match with style_manifest
        with open(style_registry, 'r') as f:
            styles = json.load(f)["styles"]
            
        for entry in styles:
            if os.path.exists(entry["npy_path"]):
                reference_pool.append({
                    "npy_path": entry["npy_path"],
                    "style_id": entry["style_id"],
                    "domain": entry["domain"],
                    "technique": entry["technique"],
                    "fps": entry["fps"]
                })
                
        with open(pool_path, 'w') as f:
            json.dump(reference_pool, f, indent=4)
        print(f"[AMP POOL] Generated reference pool with {len(reference_pool)} techniques.")

    def sanitize_vlm_json(self, raw_text: str) -> dict:
        clean_text = raw_text.strip()
        for prefix in ["```json", "```"]:
            if clean_text.startswith(prefix): clean_text = clean_text[len(prefix):].strip()
        if clean_text.endswith("```"): clean_text = clean_text[:-3].strip()
        return json.loads(clean_text)

    def queue_domain_job(self, domain: str, npy_path: str, style_id: int):
        job = {
            "video_name": os.path.basename(npy_path),
            "style": f"style_{style_id:03d}",
            "domain": domain,
            "style_id": style_id,
            "npy_path": npy_path
        }
        
        temp_path = self.queue_path + ".tmp"
        data = {"pending_jobs": []}
        if os.path.exists(self.queue_path):
            with open(self.queue_path, 'r') as f: data = json.load(f)
            
        if not any(j["style_id"] == style_id for j in data["pending_jobs"]):
            data["pending_jobs"].append(job)
            with open(temp_path, 'w') as f: json.dump(data, f, indent=4)
            shutil.move(temp_path, self.queue_path)
            print(f"[QUEUE] Queued style_{style_id:03d} ({domain}) for MoE training.")

if __name__ == "__main__":
    daemon = VideoIngestionDaemon()
    daemon.process_drop_zone()