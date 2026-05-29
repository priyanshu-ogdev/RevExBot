# revex_ext/models/moe_policy.py
import torch
import torch.nn as nn
import torch.nn.functional as F
from skrl.models.torch import GaussianMixin, Model

class RevExMoEPolicy(GaussianMixin, Model):
    def __init__(self, observation_space, action_space, device, expert_paths, 
                 router_hidden_dims=[128, 64], top_k=2, noise_scale=0.01, **kwargs):
        """
        Production-Grade MoE Policy for RevExBot.
        Handles asymmetric loading, registered normalization buffers, and gradient isolation.
        """
        super().__init__(observation_space, action_space, device, **kwargs)
        
        self.expert_names = list(expert_paths.keys())
        self.num_experts = len(self.expert_names)
        self.top_k = min(top_k, self.num_experts)
        self.noise_scale = noise_scale
        self.num_actions = action_space.shape[0]
        self.num_obs = observation_space.shape[0]
        
        # 1. Gating Network (Trainable)
        self.gating_network = nn.Sequential(
            nn.Linear(self.num_obs, router_hidden_dims[0]),
            nn.ELU(),
            nn.Linear(router_hidden_dims[0], router_hidden_dims[1]),
            nn.ELU(),
            nn.Linear(router_hidden_dims[1], self.num_experts)
        ).to(device)
        
        # 2. Expert Registry (Frozen)
        self.experts = nn.ModuleDict()
        self._build_experts(expert_paths, device)
        
        # 3. Global Log-Std (Prevents NaN during routing shifts)
        self.log_std_parameter = nn.Parameter(torch.zeros(self.num_actions))
        
        # 4. Load Balancing Coefficient
        self.load_balancing_coef = kwargs.get("load_balancing_coef", 0.01)

    def _build_experts(self, expert_paths, device):
        """Asymmetrically loads checkpoints, registers buffers, freezes weights."""
        for name, path in expert_paths.items():
            if path == "":
                # INFERENCE MODE: Create dummy buffers/experts. Master checkpoint will overwrite them.
                self.register_buffer(f"{name}_obs_mean", torch.zeros(self.num_obs, device=device))
                self.register_buffer(f"{name}_obs_std", torch.ones(self.num_obs, device=device))
                expert = self._build_expert_mlp(self.num_obs, [512, 256, 128], self.num_actions).to(device)
            else:
                checkpoint = torch.load(path, map_location=device)
                
                # 🚨 FIXED: Register normalization tensors as PyTorch Buffers
                if "running_mean_std" in checkpoint:
                    rm = checkpoint["running_mean_std"]["running_mean"].to(device)
                    rv = checkpoint["running_mean_std"]["running_var"].to(device)
                    std = torch.sqrt(rv) + 1e-8
                else:
                    rm = torch.zeros(self.num_obs, device=device)
                    std = torch.ones(self.num_obs, device=device)
                    
                self.register_buffer(f"{name}_obs_mean", rm)
                self.register_buffer(f"{name}_obs_std", std)
                    
                # Extract actor weights (strip AMP discriminator keys)
                actor_weights = {}
                for k, v in checkpoint.items():
                    if "discriminator" in k: continue
                    # Match rl_games namespace prefixing
                    key = k.replace("a2c_network.actor.", "").replace("model.", "")
                    actor_weights[key] = v
                    
                # Build & freeze expert
                expert = nn.Sequential(
                    nn.Linear(self.num_obs, 512), nn.ELU(),
                    nn.Linear(512, 256), nn.ELU(),
                    nn.Linear(256, self.num_actions)
                ).to(device)
                
                expert.load_state_dict(actor_weights, strict=False)
                expert.eval()
                for p in expert.parameters():
                    p.requires_grad = False
                    
                self.experts[name] = expert

    def _normalize_obs(self, obs, expert_name):
        """Applies expert-specific normalization and standard rl_games clamping."""
        # 🚨 FIXED: Retrieve registered buffers
        mean = getattr(self, f"{expert_name}_obs_mean", torch.zeros_like(obs))
        std = getattr(self, f"{expert_name}_obs_std", torch.ones_like(obs))
        
        norm_obs = (obs - mean) / std
        return torch.clamp(norm_obs, -5.0, 5.0) # 🚨 FIXED: Protects MLPs from outlier spikes

    def compute(self, inputs, role="policy"):
        raw_obs = inputs["states"]
        batch_size = raw_obs.shape[0]
        
        # --- ROUTING ---
        logits = self.gating_network(raw_obs)
        if self.training and self.noise_scale > 0:
            # Gumbel Softmax Noise
            gumbel = -torch.log(-torch.log(torch.rand_like(logits) + 1e-20) + 1e-20)
            logits = logits + gumbel * self.noise_scale
        probs = F.softmax(logits, dim=-1)
        
        # --- EXPERT FORWARD (Gradient Isolated) ---
        expert_means = []
        with torch.no_grad():
            for name in self.expert_names:
                norm_obs = self._normalize_obs(raw_obs, name)
                expert_means.append(self.experts[name](norm_obs))
        
        expert_means = torch.stack(expert_means, dim=1)
        
        # --- SOFT TOP-K BLENDING ---
        top_k_probs, top_k_idx = torch.topk(probs, self.top_k, dim=-1)
        top_k_probs = top_k_probs / (top_k_probs.sum(dim=-1, keepdim=True) + 1e-6)
        
        weights = torch.zeros_like(probs).unsqueeze(-1).expand(-1, -1, self.num_actions)
        weights.scatter_(1, top_k_idx.unsqueeze(-1).expand(-1, -1, self.num_actions), top_k_probs.unsqueeze(-1))
        
        final_mean = (expert_means * weights).sum(dim=1)
        
        # 🚨 FIX: Ensure the lb_loss is differentiable and returned correctly
        mean_router_probs = probs.mean(dim=0)
        lb_loss = self.num_experts * torch.sum(mean_router_probs * mean_router_probs) 
        
        # 🚨 CRITICAL: The dictionary 'extra' MUST be returned to the Agent
        return final_mean, self.log_std_parameter.expand_as(final_mean), {
            "load_balancing_loss": lb_loss
        }
    
    def act(self, inputs, role="policy"):
        """Overrides SKRL's act to inject extra metrics cleanly."""
        actions, log_probs = super().act(inputs, role)
        # Safeguard: return empty dict if compute hasn't run yet
        extra = getattr(self, "_extra_metrics", {})
        return actions, log_probs, extra