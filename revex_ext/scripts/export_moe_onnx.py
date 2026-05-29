import torch
import torch.nn as nn
import argparse
from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser(description="Export 5-Expert MoE Policy to ONNX")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to Master Router .pth")
parser.add_argument("--output", type=str, default="moe_policy.onnx", help="Output ONNX file")
parser.add_argument("--num_experts", type=int, default=5, help="Number of experts in the checkpoint")
parser.add_argument("--top_k", type=int, default=2, help="Top-K routing execution")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import revex_ext
from revex_ext.models.moe_policy import RevExMoEPolicy
from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab_tasks.utils.wrappers.skrl import SkrlVecEnvWrapper

class MoEInferenceWrapper(nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy
        
    def forward(self, obs):
        inputs = {"states": obs}
        action_mean, action_log_std, extras = self.policy.compute(inputs, role="policy")
        # Strip variance and metrics. C++ controller only needs deterministic torque targets.
        return action_mean

def main():
    print(f"📥 Loading Master Checkpoint: {args.checkpoint}")
    
    # 1. Dummy Env for tensor shapes (Using Loco to ensure scene parity)
    env_cfg = revex_ext.envs.loco.revex_loco_cfg.RevExLocoCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = SkrlVecEnvWrapper(env)
    
    # 2. Initialize Policy (Passing dummy paths; weights come from Master Checkpoint)
    dummy_paths = {k: "" for k in ["loco", "agile", "combat", "dance", "precision"]}
    policy = RevExMoEPolicy(
        observation_space=env.observation_space,
        action_space=env.action_space,
        device="cpu", # Force CPU trace for hardware-agnostic ONNX
        expert_paths=dummy_paths, # Allows init without loading individual files
        top_k=args.top_k,
        noise_scale=0.0 # Strict determinism
    )
    
    # 3. Load Master Weights
    checkpoint = torch.load(args.checkpoint, map_location="cpu")
    state_dict = checkpoint.get("policy", checkpoint.get("model", checkpoint))
    # strict=False allows loading if we skipped individual expert loading
    policy.load_state_dict(state_dict, strict=False) 
    policy.eval()
    
    # 4. Wrap & Trace
    deployment_model = MoEInferenceWrapper(policy)
    deployment_model.eval()
    dummy_obs = torch.randn(1, env.observation_space.shape[0], device="cpu")
    
    print(f"📤 Exporting 5-Expert Graph to {args.output}...")
    torch.onnx.export(
        deployment_model, dummy_obs, args.output,
        export_params=True,
        opset_version=17, 
        do_constant_folding=True, # Bakes normalizers and frozen experts directly into the binary
        input_names=['observation'],
        output_names=['action'],
        dynamic_axes={'observation': {0: 'batch_size'}, 'action': {0: 'batch_size'}}
    )
    
    print("✅ Export Complete. Ready for TensorRT Edge Deployment.")
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()