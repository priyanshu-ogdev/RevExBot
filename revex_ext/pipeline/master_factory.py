# pipeline/master_factory.py
import os
import sys
import time
import json
import yaml
import subprocess
import shutil

class MasterFactoryOrchestrator:
    def __init__(self, config_path="cfg/env_config.yaml"):
        print("[FACTORY] Initializing Master Orchestrator...")
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        self.paths = self.config['cluster_paths']
        self.drop_zone = self.paths['drop_zone']
        self.queue_path = self.paths['job_queue_json']
        
        os.makedirs(self.drop_zone, exist_ok=True)
        
        self.hardware_profile = self.config['system']['active_hardware']
        self.profiles = self.config['profiles']
        print(f"[FACTORY] Active Hardware Profile: {self.hardware_profile}")

    def _has_raw_videos(self):
        for filename in os.listdir(self.drop_zone):
            if filename.endswith(('.mp4', '.mov', '.avi')):
                return True
        return False
        
    def _has_pending_jobs(self):
        if not os.path.exists(self.queue_path):
            return False
        with open(self.queue_path, 'r') as f:
            data = json.load(f)
            return len(data.get("pending_jobs", [])) > 0

    def _atomic_save_queue(self, jobs):
        temp_path = self.queue_path + ".tmp"
        with open(temp_path, 'w') as f:
            json.dump({"pending_jobs": jobs}, f, indent=4)
        shutil.move(temp_path, self.queue_path)

    def _register_successful_expert(self, job):
        registry_path = "data/expert_registry.json"
        
        if os.path.exists(registry_path):
            with open(registry_path, 'r') as f:
                registry = json.load(f)
        else:
            registry = {"experts": []}
            
        # Prevent duplicate registry entries
        registry["experts"] = [e for e in registry["experts"] if e.get("style") != job["style"]]
        
        # RL-Games default save location structure
        expert_entry = {
            "style": job["style"],
            "domain": job["domain"],
            "latent_vector": job["latent_vector"],
            "policy_path": f"runs/{job['style']}_amp/nn/{job['style']}_amp.pth"
        }
        registry["experts"].append(expert_entry)
        
        # Atomic Write
        temp_path = registry_path + ".tmp"
        with open(temp_path, 'w') as f:
            json.dump(registry, f, indent=4)
        shutil.move(temp_path, registry_path)
        print(f"[REGISTRY] Expert '{job['style']}' permanently committed to brain map.")

    def _build_training_command(self, job):
        style = job['style']
        domain = job['domain'].lower()
        npy_path = job['npy_path']
        
        task_map = {
            "combat": "RevEx-Combat-v0",
            "dance": "RevEx-Dance-v0",
            "precision": "RevEx-Precision-v0"
        }
        task_name = task_map.get(domain, "RevEx-General-v0")
        experiment_name = f"{style}_amp"
        
        cmd = [
            "python", "scripts/train.py",
            f"--task={task_name}",
            "--phase=expert",
            "--headless",
            f"env.amp_motion_file={npy_path}", 
            f"agent_cfg.params.config.name={experiment_name}"
        ]
        
        profile = self.profiles.get(self.hardware_profile, {})
        if 'num_actors' in profile:
            cmd.append(f"--num_envs={profile['num_actors']}")
            
        return cmd

    def run_factory_loop(self):
        print("==================================================")
        print("🤖 REVEX SINGLE-MACHINE FACTORY INITIATED")
        print("==================================================")
        
        while True:
            action_taken = False
            
            # ---------------------------------------------------------
            # TICK: The Data Engine (VLM + SMPL-X + Retargeter)
            # ---------------------------------------------------------
            if self._has_raw_videos():
                print("\n[TICK] Raw videos detected. Allocating 100% GPU to Vision Daemon...")
                try:
                    subprocess.run([sys.executable, "pipeline/video_ingestion_daemon.py"], check=True)
                    print("[TICK] Vision processing complete. VRAM Flushed by Daemon.")
                    action_taken = True
                except subprocess.CalledProcessError:
                    print("[FATAL] Vision Daemon crashed. Check logs.")
                    time.sleep(10)
            
            # ---------------------------------------------------------
            # TOCK: The Simulation Engine (Isaac Lab + Router)
            # ---------------------------------------------------------
            if not action_taken and self._has_pending_jobs():
                print("\n[TOCK] Pending Isaac jobs detected. Allocating 100% GPU to RL Trainer...")
                
                with open(self.queue_path, 'r') as f:
                    queue_data = json.load(f)
                jobs = queue_data.get("pending_jobs", [])
                
                # Track if we added new experts to trigger Router update later
                new_experts_trained = False
                
                while len(jobs) > 0:
                    job = jobs[0] 
                    print(f"\n[TRAINING] Initiating Job: {job['style']}")
                    
                    cmd = self._build_training_command(job)
                    print(f"[EXEC] {' '.join(cmd)}")
                    
                    try:
                        # 1. Train the AMP Expert
                        subprocess.run(cmd, check=True)
                        print(f"[SUCCESS] Expert {job['style']} trained.")
                        
                        # 2. Register the Expert in Global Registry
                        self._register_successful_expert(job)
                        new_experts_trained = True
                        
                        # 3. Atomic Queue Update (Remove successful job)
                        jobs.pop(0)
                        self._atomic_save_queue(jobs)
                            
                    except subprocess.CalledProcessError as e:
                        print(f"[FATAL] Subprocess failed on {job['style']}. Error: {e}")
                        print("[SYSTEM] Halting TOCK queue. Job remains in queue for retry.")
                        break 
                
                # GAP 134 FIX: Train Router ONLY after all pending experts are done
                if new_experts_trained:
                    print("[DISTILL] Updating SKRL Master Router with all known experts...")
                    router_cmd = [
                        "python", "scripts/train_moe_skrl.py", # GAP 135 FIX: Correct filename
                        "--task=RevEx-Combat-v0", # Base task for obs space
                        "--phase=router",
                        "--headless"
                    ]
                    try:
                        subprocess.run(router_cmd, check=True)
                        print("[SUCCESS] Router Distillation Complete.")
                    except subprocess.CalledProcessError:
                        print("[WARN] Router Distillation failed. Proceeding anyway.")

                if len(jobs) == 0:
                    print("[TOCK] Training queue exhausted. VRAM Flushed.")
                action_taken = True
            
            # ---------------------------------------------------------
            # IDLE
            # ---------------------------------------------------------
            if not action_taken:
                print("[IDLE] Waiting for new videos in drop_zone...", end="\r")
                time.sleep(10)

if __name__ == "__main__":
    orchestrator = MasterFactoryOrchestrator()
    orchestrator.run_factory_loop()