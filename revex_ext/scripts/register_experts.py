# scripts/register_experts.py
import os
import json
import torch

def register_experts(root_dir=None):
    """
    Scans nn/ directory, validates checkpoints, and generates expert_registry.json
    for MoE Router ingestion.
    """
    if root_dir is None:
        # Derive project root from script location
        root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    nn_dir = os.path.join(root_dir, "nn")
    registry_path = os.path.join(root_dir, "revex_ext", "data", "expert_registry.json")
    
    expert_map = {
        "revex_loco_phase1": "loco",
        "revex_agile_phase2": "agile",
        "revex_skill_combat": "combat",
        "revex_skill_dance": "dance",
        "revex_skill_precision": "precision"
    }
    
    registry = {}
    
    print("🔍 Auditing Checkpoints...")
    for task_name, expert_id in expert_map.items():
        best_ckpt = os.path.join(nn_dir, f"{task_name}_best.pth")
        
        if os.path.exists(best_ckpt):
            try:
                # 🚨 FIX: map_location for cross-device compatibility
                ckpt = torch.load(best_ckpt, map_location='cpu')
                if "model" in ckpt or "policy" in ckpt:
                    # 🚨 FIX: Verify normalization stats for MoE
                    if "running_mean_std" not in ckpt:
                        print(f"⚠️ Warning: {best_ckpt} missing running_mean_std")
                    registry[expert_id] = os.path.abspath(best_ckpt)
                    print(f"✅ Registered {expert_id} from {best_ckpt}")
                else:
                    print(f"⚠️ Warning: {best_ckpt} missing model keys")
            except Exception as e:
                print(f"❌ Error loading {best_ckpt}: {e}")
        else:
            # 🚨 FIX: Fallback to last checkpoint if best not found
            last_ckpt = os.path.join(nn_dir, f"{task_name}.pth")
            if os.path.exists(last_ckpt):
                try:
                    ckpt = torch.load(last_ckpt, map_location='cpu')
                    if "model" in ckpt or "policy" in ckpt:
                        registry[expert_id] = os.path.abspath(last_ckpt)
                        print(f"✅ Registered {expert_id} from {last_ckpt} (fallback)")
                except Exception as e:
                    print(f"❌ Error loading {last_ckpt}: {e}")
            else:
                print(f"❌ Missing checkpoint for {task_name}")

    # Ensure output directory exists
    os.makedirs(os.path.dirname(registry_path), exist_ok=True)
    
    with open(registry_path, 'w') as f:
        json.dump(registry, f, indent=4)
    print(f"🚀 Registry sealed at {registry_path}")
    return registry

if __name__ == "__main__":
    register_experts()