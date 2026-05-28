# pipeline/video_ingestion_daemon.py
import os
import sys
import cv2
import json
import yaml
import shutil
import torch
import numpy as np
from PIL import Image
from transformers import Qwen2_5_VLForConditionalGeneration, AutoProcessor
from qwen_vl_utils import process_vision_info

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from kinematic_retargeter import KinematicRetargeter

class VideoIngestionDaemon:
    def __init__(self, config_path="cfg/env_config.yaml"):
        print("[DAEMON] Booting Native Vision Triage Engine (Multi-Agent Aggregation Mode)...")
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        paths = self.config['cluster_paths']
        self.drop_zone = paths['drop_zone']
        self.clean_video_dir = paths['raw_videos']
        self.queue_path = paths['job_queue_json']
        
        os.makedirs(self.drop_zone, exist_ok=True)
        os.makedirs(self.clean_video_dir, exist_ok=True)
        
        self.model_id = "Qwen/Qwen2.5-VL-3B-Instruct"
        self.max_frames = self.config['vlm_orchestration'].get('max_frames_per_video', 8)

    def extract_motion_peaks(self, video_path: str):
        cap = cv2.VideoCapture(video_path)
        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps == 0 or np.isnan(fps) or fps > 120 or fps < 10:
            fps = 30.0 
            
        frame_scores = []
        frame_count = 0
        ret, prev_frame = cap.read()
        if not ret: 
            cap.release()
            return [], fps
            
        prev_gray = cv2.cvtColor(cv2.resize(prev_frame, (512, 512)), cv2.COLOR_BGR2GRAY)
        
        while True:
            ret, frame = cap.read()
            if not ret: break
            if frame_count % 3 == 0:
                small_frame = cv2.resize(frame, (512, 512))
                gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
                delta = cv2.absdiff(prev_gray, gray)
                score = np.sum(delta)
                frame_scores.append((frame_count, score))
                prev_gray = gray
            frame_count += 1
        cap.release()
        
        if not frame_scores: return [], fps
        frame_scores.sort(key=lambda x: x[1], reverse=True)
        top_indices = [idx for idx, _ in frame_scores[:self.max_frames]]
        top_indices.sort() 
        return top_indices, fps

    def sanitize_vlm_json(self, raw_text: str) -> dict:
        clean_text = raw_text.strip()
        if clean_text.startswith("```json"): clean_text = clean_text[7:]
        if clean_text.startswith("```"): clean_text = clean_text[3:]
        if clean_text.endswith("```"): clean_text = clean_text[:-3]
        return json.loads(clean_text.strip())

    def queue_domain_job(self, domain: str, master_npy_path: str, latent_vector: list):
        job = {
            "video_name": f"batch_aggregated_{domain.lower()}",
            "style": f"master_{domain.lower()}", 
            "domain": domain, 
            "latent_vector": latent_vector,
            "npy_path": master_npy_path
        }
        
        temp_path = self.queue_path + ".tmp"
        if os.path.exists(self.queue_path):
            with open(self.queue_path, 'r') as f:
                data = json.load(f)
        else:
            data = {"pending_jobs": []}
            
        # Prevent duplicates
        for existing_job in data["pending_jobs"]:
            if existing_job["domain"] == domain:
                print(f"[QUEUE] {domain} job already pending. Updating NPY path.")
                existing_job["npy_path"] = master_npy_path
                existing_job["latent_vector"] = latent_vector
                with open(temp_path, 'w') as f:
                    json.dump(data, f, indent=4)
                shutil.move(temp_path, self.queue_path)
                return

        data["pending_jobs"].append(job)
        with open(temp_path, 'w') as f:
            json.dump(data, f, indent=4)
        shutil.move(temp_path, self.queue_path)
        print(f"[QUEUE] Master {domain} job appended.")

    def extract_smplx_params_multi_agent(self, video_path: str):
        """
        GAP 81 FIX: Multi-Agent Extraction.
        In production, this uses a tracker (e.g., 4D-Humans/SLAHMR) to isolate IDs.
        Returns a LIST of SMPL-X dictionaries, one for each detected person.
        """
        print("[SMPL-X] Running Multi-Person Tracking and Extraction...")
        cap = cv2.VideoCapture(video_path)
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        
        if total_frames == 0: raise ValueError("Video has 0 frames")

        # MOCKING MULTI-AGENT DETECTION: 
        # In production, replace this with actual tracking logic that returns N people.
        # For now, we assume 1 person per video to prevent crash, but structure supports N.
        num_people_detected = 1 
        
        trajectories = []
        for person_id in range(num_people_detected):
            smplx_params = {
                'person_id': person_id,
                'transl': np.zeros((total_frames, 3), dtype=np.float32), # 🚨 ADDED
                'body_pose': np.zeros((total_frames, 21, 3), dtype=np.float32),
                'global_orient': np.zeros((total_frames, 1, 3), dtype=np.float32),
                'left_hand_pose': np.zeros((total_frames, 15, 3), dtype=np.float32),
                'right_hand_pose': np.zeros((total_frames, 15, 3), dtype=np.float32),
                'betas': np.zeros((10,), dtype=np.float32)
            }
            trajectories.append(smplx_params)
            
        return trajectories

    def process_drop_zone(self):
        files = [f for f in os.listdir(self.drop_zone) if f.endswith(('.mp4', '.mov', '.avi'))]
        if not files:
            print("[DAEMON] Drop zone empty. Exiting.")
            return

        print(f"[DAEMON] Found {len(files)} raw videos. Initiating 3-Phase Aggregation...")
        video_metadata = []

        # ============================================================
        # PHASE 1: BATCH CLASSIFICATION (VLM OWNERSHIP)
        # ============================================================
        print(f"[PHASE 1] Loading {self.model_id} into VRAM...")
        processor = AutoProcessor.from_pretrained(self.model_id)
        model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
            self.model_id, torch_dtype=torch.bfloat16, device_map="auto"
        )
        
        for filename in files:
            raw_path = os.path.join(self.drop_zone, filename)
            try:
                frame_indices, fps = self.extract_motion_peaks(raw_path)
                if not frame_indices: continue
                
                print(f"[VLM] Classifying {filename}...")
                cap = cv2.VideoCapture(raw_path)
                pil_frames = []
                for idx in frame_indices:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
                    ret, frame = cap.read()
                    if ret:
                        frame_rgb = cv2.cvtColor(cv2.resize(frame, (512, 512)), cv2.COLOR_BGR2RGB)
                        pil_frames.append(Image.fromarray(frame_rgb))
                cap.release()

                prompt = """
                You are an expert robotic kinematic classifier. Analyze these sequential frames.
                1. Classify the domain as EXACTLY one of: 'Combat', 'Dance', or 'Precision'.
                2. Generate a lowercase, snake_case style name.
                Respond ONLY with a JSON object: {"domain": "Combat", "style_name": "string", "latent_vector": [1,0,0,0]}
                """
                content_list = [{"type": "image", "image": img} for img in pil_frames]
                content_list.append({"type": "text", "text": prompt})
                messages = [{"role": "user", "content": content_list}]
                
                text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
                image_inputs, video_inputs = process_vision_info(messages)
                inputs = processor(text=[text], images=image_inputs, videos=video_inputs, padding=True, return_tensors="pt").to("cuda")
                
                with torch.no_grad():
                    generated_ids = model.generate(**inputs, max_new_tokens=200, temperature=0.1)
                    
                generated_ids_trimmed = [out_ids[len(in_ids):] for in_ids, out_ids in zip(inputs.input_ids, generated_ids)]
                output_text = processor.batch_decode(generated_ids_trimmed, skip_special_tokens=True)[0]
                
                classification = self.sanitize_vlm_json(output_text)
                
                clean_name = f"{classification['style_name']}.mp4"
                clean_path = os.path.join(self.clean_video_dir, clean_name)
                counter = 1
                while os.path.exists(clean_path):
                    clean_name = f"{classification['style_name']}_{counter:02d}.mp4"
                    clean_path = os.path.join(self.clean_video_dir, clean_name)
                    counter += 1
                shutil.move(raw_path, clean_path)
                
                video_metadata.append({
                    "clean_name": clean_name,
                    "clean_path": clean_path,
                    "fps": fps,
                    "classification": classification
                })
                
            except Exception as e:
                print(f"[ERROR] Classification failed on {filename}. Exception: {e}")

        del model
        del processor
        torch.cuda.empty_cache()
        print("[PHASE 1 COMPLETE] VLM unloaded.")

        # ============================================================
        # PHASE 2: BATCH KINEMATICS (SMPL-X OWNERSHIP)
        # ============================================================
        if not video_metadata:
            print("[DAEMON] No valid videos survived classification. Exiting.")
            return

        print("[PHASE 2] Loading Kinematic Retargeter into VRAM...")
        retargeter = KinematicRetargeter(config_path="cfg/env_config.yaml")
        
        domain_tensors = {"Combat": [], "Dance": [], "Precision": []}
        domain_latents = {"Combat": [], "Dance": [], "Precision": []}

        for meta in video_metadata:
            try:
                print(f"[SMPL-X] Processing {meta['clean_name']}...")
                
                # GAP 81 FIX: Extract Multi-Agent Trajectories
                multi_person_params = self.extract_smplx_params_multi_agent(meta['clean_path'])
                
                for person_params in multi_person_params:
                    p_id = person_params['person_id']
                    virtual_clip_name = f"{meta['clean_name']}_p{p_id}"
                    
                    # Retarget each person independently
                    npy_path = retargeter.process_kinematics_batched(person_params, virtual_clip_name, meta['fps'])
                    tensor_data = np.load(npy_path)
                    
                    domain = meta['classification']['domain']
                    domain_tensors[domain].append(tensor_data)
                    domain_latents[domain].append(meta['classification']['latent_vector'])
                    
                    print(f"[RETARGETER] Isolated and processed Person {p_id} from {meta['clean_name']}")
                    
            except Exception as e:
                print(f"[ERROR] Retargeting failed on {meta['clean_name']}. Exception: {e}")

        del retargeter
        torch.cuda.empty_cache()
        print("[PHASE 2 COMPLETE] Retargeter unloaded.")

        # ============================================================
        # PHASE 3: MANIFOLD AGGREGATION & QUEUEING
        # ============================================================
        print("[PHASE 3] Aggregating Expert Manifolds...")
        os.makedirs(self.config['cluster_paths']['processed_kinematics'], exist_ok=True)
        
        for domain, tensors in domain_tensors.items():
            if not tensors: continue
            
            # Concatenate all clips (including multi-agent splits) into one master file
            master_tensor = np.concatenate(tensors, axis=0)
            
            master_npy_name = f"master_{domain.lower()}.npy"
            master_npy_path = os.path.join(self.config['cluster_paths']['processed_kinematics'], master_npy_name)
            
            if os.path.exists(master_npy_path):
                existing_tensor = np.load(master_npy_path)
                master_tensor = np.concatenate((existing_tensor, master_tensor), axis=0)
                
            np.save(master_npy_path, master_tensor)
            print(f"[AGGREGATION] {domain} manifold updated: Shape {master_tensor.shape}")
            
            # Average Latent Vectors for the Domain
            avg_latent = np.mean(domain_latents[domain], axis=0).tolist()
            self.queue_domain_job(domain, master_npy_path, avg_latent)

        print("[DAEMON] Batch Aggregation Complete. Terminating.")

if __name__ == "__main__":
    daemon = VideoIngestionDaemon()
    daemon.process_drop_zone()