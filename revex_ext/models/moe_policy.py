import torch
import torch.nn as nn
import torch.nn.functional as F

# SKRL 1.2.0+ Native Mixins
from skrl.models.torch import GaussianMixin, DeterministicMixin

class RevExMoEPolicy(GaussianMixin, nn.Module):
    def __init__(self, observation_space, action_space, device, 
                 num_experts=3, expert_hidden_dims=[256, 128], 
                 router_hidden_dims=[128, 64], top_k=2, noise_scale=0.01):
        """
        SOTA Mixture of Experts Policy for RevExBot.
        Optimized for 16GB VRAM (Dense Routing with Auxiliary Loss).
        """
        nn.Module.__init__(self)
        GaussianMixin.__init__(self, clip_actions=True, clip_log_std=True, min_log_std=-20, max_log_std=2)
        
        self.num_experts = num_experts
        self.top_k = top_k
        self.noise_scale = noise_scale
        
        self.num_obs = observation_space.shape[0]
        self.num_actions = action_space.shape[0]
        
        # 1. The Differentiable Gating Network
        self.router = nn.Sequential(
            nn.Linear(self.num_obs, router_hidden_dims[0]),
            nn.ELU(),
            nn.Linear(router_hidden_dims[0], router_hidden_dims[1]),
            nn.ELU(),
            nn.Linear(router_hidden_dims[1], num_experts)
        )
        
        # 2. The Domain Experts
        self.experts = nn.ModuleList([
            self._build_expert(self.num_obs, expert_hidden_dims, self.num_actions)
            for _ in range(num_experts)
        ])
        
        # 3. Global Action Variance (Prevents NaN explosions during domain swaps)
        self.log_std_parameter = nn.Parameter(torch.zeros(self.num_actions))
        
        self.apply(self._init_weights)

    def _build_expert(self, input_dim, hidden_dims, output_dim):
        layers = []
        dims = [input_dim] + hidden_dims
        for i in range(len(dims) - 1):
            layers.append(nn.Linear(dims[i], dims[i+1]))
            layers.append(nn.ELU())
        layers.append(nn.Linear(hidden_dims[-1], output_dim)) 
        return nn.Sequential(*layers)

    def _init_weights(self, m):
        if isinstance(m, nn.Linear):
            nn.init.orthogonal_(m.weight, gain=0.01)
            nn.init.constant_(m.bias, 0)

    def compute(self, inputs, role=""):
        obs = inputs["states"]
        batch_size = obs.shape[0]
        
        # --- ROUTING ---
        logits = self.router(obs)
        
        if self.training and self.noise_scale > 0:
            noise = -torch.log(-torch.log(torch.rand_like(logits) + 1e-20) + 1e-20)
            logits = logits + noise * self.noise_scale
            
        router_probs = F.softmax(logits, dim=-1)
        
        # --- SOFT TOP-K SELECTION ---
        top_k_probs, top_k_indices = torch.topk(router_probs, self.top_k, dim=-1)
        top_k_probs = top_k_probs / top_k_probs.sum(dim=-1, keepdim=True) 
        
        # --- EXPERT EXECUTION ---
        # Evaluate all experts (Dense computation is faster for 3 small MLPs on a 4070 Ti)
        all_actions = torch.stack([expert(obs) for expert in self.experts], dim=1) 
        
        # --- ACTION BLENDING ---
        weights = torch.zeros_like(router_probs).unsqueeze(-1).expand(-1, -1, self.num_actions)
        weights.scatter_(1, top_k_indices.unsqueeze(-1).expand(-1, -1, self.num_actions), top_k_probs.unsqueeze(-1))
        
        final_mean = (all_actions * weights).sum(dim=1)
        
        # --- AUXILIARY LOAD BALANCING LOSS ---
        # We calculate it here and pass it out via the SKRL extras dictionary
        # Mean probability routed to each expert
        mean_router_probs = router_probs.mean(dim=0)
        # Fraction of batch actually assigned to each expert (Top-1 for balancing math)
        top_1_indices = top_k_indices[:, 0]
        expert_fractions = torch.bincount(top_1_indices, minlength=self.num_experts).float() / batch_size
        
        # Differentiable Load Balancing Penalty
        load_balancing_loss = self.num_experts * torch.sum(mean_router_probs * expert_fractions)
        
        return final_mean, self.log_std_parameter.expand_as(final_mean), {
            "router_probs": router_probs,
            "load_balancing_loss": load_balancing_loss
        }

# ---------------------------------------------------------
# THE ASYMMETRIC CRITIC (God-Mode Observer)
# ---------------------------------------------------------
class RevExCritic(DeterministicMixin, nn.Module):
    def __init__(self, privileged_observation_space, device):
        """
        Critic requires the PRIVILEGED observation space (148 + physics states)
        to properly evaluate the value function in an asymmetric setup.
        """
        nn.Module.__init__(self)
        DeterministicMixin.__init__(self, clip_actions=False)
        
        num_obs = privileged_observation_space.shape[0]
        
        self.net = nn.Sequential(
            nn.Linear(num_obs, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 1)
        )
        self.apply(self._init_weights)

    def _init_weights(self, m):
        if isinstance(m, nn.Linear):
            nn.init.orthogonal_(m.weight, gain=1.0)
            nn.init.constant_(m.bias, 0)

    def compute(self, inputs, role=""):
        # SKRL automatically routes the 'states' key based on the environment setup.
        # If is_asymmetric=True, inputs["states"] will contain the privileged tensor.
        return self.net(inputs["states"]), {}