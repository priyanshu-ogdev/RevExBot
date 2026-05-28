import torch
import torch.nn as nn
import argparse
from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser(description="Export MoE Policy to ONNX")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to .pt checkpoint")
parser.add_argument("--output", type=str, default="moe_policy.onnx", help="Output ONNX file")
parser.add_argument("--num_experts", type=int, default=3, help="Number of experts")
parser.add_argument("--top_k", type=int, default=2, help="Top-K routing")

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# Imports
import revex_ext
from revex_ext.models.moe_policy import RevExMoEPolicy
from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab_tasks.utils.wrappers.skrl import SkrlVecEnvWrapper

# 🚨 GAP 136 FIX: The Inference Wrapper
class MoEInferenceWrapper(nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy
        
    def forward(self, obs):
        # 1. Package the raw tensor into the SKRL dictionary format
        inputs = {"states": obs}
        
        # 2. Execute the MoE Policy
        action_mean, action_log_std, extras = self.policy.compute(inputs, role="policy")
        
        # 3. Strip the variance and extras. Return ONLY the deterministic action.
        return action_mean

def main():
    print(f"📥 Loading Checkpoint: {args.checkpoint}")
    
    # 1. Create Dummy Env to get obs/action spaces
    env_cfg = revex_ext.envs.skills.revex_combat_cfg.RevExCombatEnvCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = SkrlVecEnvWrapper(env)
    
    # 2. Initialize Policy
    policy = RevExMoEPolicy(
        observation_space=env.observation_space,
        action_space=env.action_space,
        device="cpu", # Export on CPU for universal hardware compatibility
        num_experts=args.num_experts,
        top_k=args.top_k,
        noise_scale=0.0 # Disable Gumbel noise for deterministic inference
    )
    
    # 3. Load Weights Safely
    checkpoint = torch.load(args.checkpoint, map_location="cpu")
    # Handle SKRL's nested state_dict format
    if "policy" in checkpoint:
        policy.load_state_dict(checkpoint["policy"])
    elif "model" in checkpoint:
        policy.load_state_dict(checkpoint["model"])
    else:
        policy.load_state_dict(checkpoint)
        
    policy.eval()
    
    # 4. Wrap the Policy
    deployment_model = MoEInferenceWrapper(policy)
    deployment_model.eval()
    
    # 5. Define Dummy Input (Batch Size 1 for the physical robot)
    dummy_obs = torch.randn(1, env.observation_space.shape[0], device="cpu")
    
    # 6. Export to ONNX
    print(f"📤 Exporting graph to {args.output}...")
    torch.onnx.export(
        deployment_model,               # The wrapped model
        dummy_obs,                      # Raw tensor input
        args.output,
        export_params=True,
        opset_version=17,               # Supports advanced indexing/scattering
        do_constant_folding=True,       # Bakes frozen expert weights into the graph
        input_names=['observation'],
        output_names=['action'],        # Clean single output for your C++ controller
        dynamic_axes={
            'observation': {0: 'batch_size'},
            'action': {0: 'batch_size'}
        }
    )
    
    print("✅ Export Complete. Graph is sealed for TensorRT/NCNN.")
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()