# pipeline/master_factory.py
import os
import sys
import time
import json
import yaml
import subprocess
import shutil
import glob

class MasterFactoryOrchestrator:
    def __init__(self, config_path="cfg/env_config.yaml"):
        print("[FACTORY] Initializing Master Orchestrator (Domain-Expert Architecture)...")
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        self.paths = self.config['cluster_paths']
        self.drop_zone = self.paths['drop_zone']
        self.queue_path = self.paths['job_queue_json']
        self.weights_dir = self.paths['weights_dir']
        self.registry_path = self.paths['registry_json']
        
        os.makedirs(self.drop_zone, exist_ok=True)
        os.makedirs(self.weights_dir, exist_ok=True)
        
        self.hardware_profile = self.config['system']['active_hardware']
        self.profiles = self.config['profiles']
        print(f"[FACTORY] Active Hardware Profile: {self.hardware_profile}")

    def _has_raw_videos(self):
        for filename in os.listdir(self.drop_zone):
            if filename.endswith(('.mp4', '.mov', '.avi')):
                return True
        return False
        
    def _get_pending_domains(self):
        """Reads queue and groups jobs by Domain instead of individual techniques."""
        if not os.path.exists(self.queue_path):
            return []
        with open(self.queue_path, 'r') as f:
            data = json.load(f)
            jobs = data.get("pending_jobs", [])
            
        # Extract unique domains that need retraining
        domains = list(set([job['domain'] for job in jobs]))
        return domains, jobs

    def _clear_domain_from_queue(self, completed_domain, all_jobs):
        """Removes all jobs belonging to a successfully trained Domain."""
        remaining_jobs = [job for job in all_jobs if job['domain'] != completed_domain]
        temp_path = self.queue_path + ".tmp"
        with open(temp_path, 'w') as f:
            json.dump({"pending_jobs": remaining_jobs}, f, indent=4)
        shutil.move(temp_path, self.queue_path)
        return remaining_jobs

    def _archive_and_register_expert(self, domain):
        """Finds the newest .pth, moves it to the vault, and updates the MoE Router registry."""
        task_name = f"revex_{domain.lower()}_phase3" # Matches YAML config name
        run_dir = f"runs/{task_name}/nn"
        
        if not os.path.exists(run_dir):
            print(f"[WARN] No weights found for {domain} in {run_dir}")
            return False
            
        # Find the latest .pth file (RL-Games output)
        pth_files = glob.glob(os.path.join(run_dir, "*.pth"))
        if not pth_files:
            return False
            
        latest_pth = max(pth_files, key=os.path.getctime)
        
        # Move to immutable vault with versioning
        version = int(time.time())
        vault_path = os.path.join(self.weights_dir, f"expert_{domain.lower()}_v{version}.pth")
        shutil.copy2(latest_pth, vault_path)
        
        # Update Registry for the SKRL Router
        if os.path.exists(self.registry_path):
            with open(self.registry_path, 'r') as f: registry = json.load(f)
        else:
            registry = {"experts": []}
            
        # Remove old entry for this domain, insert new one
        registry["experts"] = [e for e in registry["experts"] if e.get("domain") != domain]
        registry["experts"].append({
            "domain": domain,
            "policy_path": vault_path,
            "latent_conditioned": True # Flags SKRL to expect Style IDs
        })
        
        temp_path = self.registry_path + ".tmp"
        with open(temp_path, 'w') as f: json.dump(registry, f, indent=4)
        shutil.move(temp_path, self.registry_path)
        
        print(f"[REGISTRY] Domain '{domain}' committed to Master Brain Map: {vault_path}")
        return True

    def _build_training_command(self, domain):
        task_map = {
            "Combat": "RevEx-Combat-v0",
            "Dance": "RevEx-Dance-v0",
            "Precision": "RevEx-Precision-v0"
        }
        task_name = task_map.get(domain, "RevEx-General-v0")
        
        cmd = [
            "python", "scripts/train.py",
            f"--task={task_name}",
            "--phase=expert",
            "--headless"
            # 🚨 REMOVED: env.amp_motion_file. The Env now reads reference_pool.json natively.
        ]
        return cmd

    def run_factory_loop(self):
        print("==================================================")
        print("🤖 REVEX SINGLE-NODE MASTER FORGE INITIATED")
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
                    print("[TICK] Vision processing complete. Subprocess exit naturally flushed VRAM.")
                    action_taken = True
                except subprocess.CalledProcessError:
                    print("[FATAL] Vision Daemon crashed. Check logs.")
                    time.sleep(10)
            
            # ---------------------------------------------------------
            # TOCK: The Simulation Engine (Isaac Lab Domain Training)
            # ---------------------------------------------------------
            domains_to_train, all_jobs = self._get_pending_domains()
            
            if not action_taken and domains_to_train:
                print(f"\n[TOCK] Pending data for Domains: {domains_to_train}. Igniting RL Forge...")
                new_experts_trained = False
                
                for domain in domains_to_train:
                    print(f"\n[TRAINING] Upgrading Domain Expert: {domain}")
                    cmd = self._build_training_command(domain)
                    print(f"[EXEC] {' '.join(cmd)}")
                    
                    try:
                        # 1. Train the Domain Expert (e.g., Combat) on the entire reference_pool.json
                        subprocess.run(cmd, check=True)
                        print(f"[SUCCESS] Domain Expert '{domain}' training completed.")
                        
                        # 2. Extract weights to vault & register for MoE
                        self._archive_and_register_expert(domain)
                        new_experts_trained = True
                        
                        # 3. Clear all jobs related to this domain from the queue
                        all_jobs = self._clear_domain_from_queue(domain, all_jobs)
                            
                    except subprocess.CalledProcessError as e:
                        print(f"[FATAL] RL Engine failed on Domain '{domain}'. Error: {e}")
                        print("[SYSTEM] Skipping domain. Jobs remain in queue for retry.")
                
                # ---------------------------------------------------------
                # APEX: The SKRL MoE Router Distillation
                # ---------------------------------------------------------
                if new_experts_trained:
                    print("\n[DISTILL] Updating SKRL Master Router with upgraded Domain Experts...")
                    router_cmd = [
                        "python", "scripts/train_moe_skrl.py",
                        "--task=RevEx-Combat-v0", 
                        "--phase=router",
                        "--headless"
                    ]
                    try:
                        subprocess.run(router_cmd, check=True)
                        print("[SUCCESS] Router Distillation Complete. System is deployment-ready.")
                    except subprocess.CalledProcessError:
                        print("[WARN] Router Distillation failed. Check SKRL logs.")

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