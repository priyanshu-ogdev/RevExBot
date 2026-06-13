"""
Export the RevEx ASE policy (actor only) to ONNX for edge deployment.
The exported model expects:
    actor_obs_seq : (batch, history_length, obs_dim)
    style_code    : (batch, latent_dim)
and outputs the deterministic, bounded action (batch, act_dim) in [-1, 1].
"""
import argparse
import torch
import torch.nn as nn

from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser(description="Export RevEx ASE policy to ONNX")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to the trained policy checkpoint (.pt)")
parser.add_argument("--output", type=str, default="ase_policy.onnx", help="Output ONNX file name")
parser.add_argument("--phase", type=int, default=2, choices=[1, 2], help="Phase of the checkpoint (1 = base loco, 2 = ASE style)")
AppLauncher.add_app_launcher_args(parser)
args, _ = parser.parse_known_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

from revex_ext.envs.revex_ase_env import RevExAseEnv
from revex_ext.envs.revex_ase_env_cfg import RevExAseEnvCfg
from revex_ext.agents.ase_policy import ASEHistoryPolicy


class ActorInferenceWrapper(nn.Module):
    """Wraps the actor part of ASEHistoryPolicy for deterministic, hardware‑safe inference."""
    def __init__(self, policy: ASEHistoryPolicy):
        super().__init__()
        # Copy only the necessary actor sub‑networks
        self.actor_history = policy.actor_history
        self.actor_trunk = policy.actor_trunk
        self.mean_head_body = policy.mean_head_body
        self.mean_head_hand = policy.mean_head_hand

    def forward(self, actor_obs_seq: torch.Tensor, style_code: torch.Tensor) -> torch.Tensor:
        # Actor context (Causal CNN processing)
        act_ctx = self.actor_history(actor_obs_seq, style_code)
        actor_features = self.actor_trunk(act_ctx)

        # Split‑head mean calculation
        mean_body = self.mean_head_body(actor_features)
        mean_hand = self.mean_head_hand(actor_features)
        mean = torch.cat([mean_body, mean_hand], dim=-1)

        # HARDWARE SAFETY LOCK:
        # Bake the Tanh squashing directly into the ONNX graph.
        # This guarantees the edge controller receives strictly bounded [-1.0, 1.0] actions,
        # preventing catastrophic servo over‑extensions.
        return torch.tanh(mean)


def main():
    print(f"📥 Loading checkpoint: {args.checkpoint}")

    # 1. Create a dummy environment to obtain observation dimensions.
    #    We use phase=1 to avoid needing motion library; the env is just for shapes.
    env_cfg = RevExAseEnvCfg(phase=1)
    env_cfg.scene.num_envs = 1
    env = RevExAseEnv(cfg=env_cfg)

    obs_dict = env.reset()
    actor_obs_seq = obs_dict["policy"]                 # (1, history_length, obs_dim)

    # 🚨 FIX: style code is stored in ase_data, not a separate observation key
    style_code = env.unwrapped.extras["ase_data"]["z"]  # (1, latent_dim)

    actor_obs_dim = actor_obs_seq.shape[-1]            # single‑step observation size
    critic_obs_dim = obs_dict["critic"].shape[-1]
    style_dim = style_code.shape[-1]
    act_dim = env.action_space.shape[0]

    # 2. Instantiate the full policy and load weights
    policy = ASEHistoryPolicy(
        obs_dim=actor_obs_dim,
        critic_obs_dim=critic_obs_dim,
        act_dim=act_dim,
        latent_dim=style_dim
    )
    checkpoint = torch.load(args.checkpoint, map_location="cpu")
    policy.load_state_dict(checkpoint["policy"])
    policy.eval()

    # 3. Create the actor‑only wrapper for export
    actor_model = ActorInferenceWrapper(policy)
    actor_model.eval()

    # 4. Dummy inputs for tracing
    dummy_obs_seq = torch.randn(1, actor_obs_seq.shape[1], actor_obs_dim)
    dummy_style = torch.randn(1, style_dim)

    print(f"📤 Exporting actor (ONNX) to {args.output} ...")
    torch.onnx.export(
        actor_model,
        (dummy_obs_seq, dummy_style),
        args.output,
        export_params=True,
        opset_version=17,
        do_constant_folding=True,
        input_names=["actor_obs_seq", "style_code"],
        output_names=["action"],
        dynamic_axes={
            "actor_obs_seq": {0: "batch_size"},
            "style_code": {0: "batch_size"},
            "action": {0: "batch_size"}
        }
    )
    print("✅ Export complete. Ready for edge deployment (TensorRT / ONNX Runtime).")

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()