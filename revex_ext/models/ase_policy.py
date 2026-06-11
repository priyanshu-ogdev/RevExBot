"""
Universal Motion Model (UMM) Policy Architecture for RevExBot.
Features: Split-Head Hands/Body, Zoned Exploration, Causal CNN History, Asymmetric Critic.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal, TransformedDistribution, TanhTransform
import numpy as np
from typing import Tuple

# ========================================================================
# 1. Initialization Core
# ========================================================================
def init_weights(m):
    """Applies RL-specific orthogonal and Kaiming initialization."""
    if isinstance(m, nn.Linear):
        nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
        if m.bias is not None:
            nn.init.constant_(m.bias, 0)
    elif isinstance(m, nn.Conv1d):
        nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
        if m.bias is not None:
            nn.init.constant_(m.bias, 0)

# ========================================================================
# 2. Variational Motion Encoder (VIB)
# ========================================================================
class VariationalMotionEncoder(nn.Module):
    """Compresses mocap sequences into a smooth, continuous latent manifold (z)."""
    def __init__(self, mocap_dim: int, latent_dim: int = 16):
        super().__init__()
        self.conv1 = nn.Conv1d(mocap_dim, 64, kernel_size=3, padding=1)
        self.conv2 = nn.Conv1d(64, 128, kernel_size=3, padding=1)
        self.pool = nn.AdaptiveAvgPool1d(1)
        
        self.mu_net = nn.Linear(128, latent_dim)
        self.logvar_net = nn.Linear(128, latent_dim)
        
        self.apply(init_weights)
        nn.init.orthogonal_(self.mu_net.weight, gain=0.01)
        nn.init.orthogonal_(self.logvar_net.weight, gain=0.01)

    def forward(self, mocap_window: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        # Switch (Batch, Seq, Dim) -> (Batch, Dim, Seq) for 1D CNN
        x = mocap_window.transpose(1, 2)
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = self.pool(x).squeeze(-1)

        mu = self.mu_net(x)
        # Safe numerical clamp for logvar (prevents exp() overflow)
        logvar = torch.clamp(self.logvar_net(x), -10, 10)
        std = torch.exp(0.5 * logvar)
        
        # Reparameterization Trick for differentiable sampling
        eps = torch.randn_like(std)
        z = mu + eps * std
        return z, mu, logvar

# ========================================================================
# 3. Strictly Causal Temporal Encoder
# ========================================================================
class CausalHistoryEncoder(nn.Module):
    """Grants the actor short-term momentum memory without future-leaking."""
    def __init__(self, obs_dim: int, latent_dim: int = 16):
        super().__init__()
        self.proj = nn.Linear(obs_dim + latent_dim, 128)
        self.conv1 = nn.Conv1d(128, 256, kernel_size=3, padding=0)
        self.conv2 = nn.Conv1d(256, 256, kernel_size=3, padding=0)
        self.apply(init_weights)

    def forward(self, obs_seq: torch.Tensor, z: torch.Tensor) -> torch.Tensor:
        seq_len = obs_seq.size(1)
        # Expand the style code to match the history sequence length
        z_exp = z.unsqueeze(1).expand(-1, seq_len, -1)
        
        x = torch.cat([obs_seq, z_exp], dim=-1)
        x = self.proj(x).transpose(1, 2)

        # STRICT CAUSALITY: Pad the temporal dimension on the LEFT ONLY
        x = F.pad(x, (2, 0))
        x = F.relu(self.conv1(x))
        
        x = F.pad(x, (2, 0)) # Prevents sequence shrink & future leakage
        x = F.relu(self.conv2(x))

        # Extract the final, context-rich timestep
        return x[:, :, -1]

# ========================================================================
# 4. The Universal Motion Model (Actor-Critic)
# ========================================================================
class ASEHistoryPolicy(nn.Module):
    """
    Unified brain combining Asymmetric Actor-Critic and Split-Head Hardware Constraints.
    - Actor: Causal CNN over observation history + style code (Split-Head output).
    - Critic: Flat MLP over privileged current state + style code.
    """
    def __init__(self, obs_dim: int, critic_obs_dim: int, latent_dim: int = 16):
        super().__init__()
        
        # ---------------------------------------------------------
        # ACTOR: Temporal CNN
        # ---------------------------------------------------------
        self.actor_history = CausalHistoryEncoder(obs_dim, latent_dim)
        self.actor_trunk = nn.Sequential(
            nn.Linear(256, 1024), nn.ELU(),
            nn.Linear(1024, 512), nn.ELU(),
        )
        
        # SPLIT-HEAD: Protects delicate finger gradients from massive torso gradients
        # Total DOFs = 39 (14 Legs + 3 Torso + 10 Arms = 27 Body | 12 Hands)
        self.mean_head_body = nn.Linear(512, 27)
        self.mean_head_hand = nn.Linear(512, 12)
        
        # ZONED EXPLORATION: Forces arms to compensate for rigid 1D-hook fingers
        self.log_std_loco = nn.Parameter(torch.full((17,), -0.847)) # Legs & Torso (std ≈ 0.6)
        self.log_std_arms = nn.Parameter(torch.full((10,), 0.0))    # Arms (std ≈ 1.0 - HIGH)
        self.log_std_hands = nn.Parameter(torch.full((12,), -2.19)) # Hands (std ≈ 0.1 - LOW)

        # ---------------------------------------------------------
        # CRITIC: Asymmetric Flat MLP (No History)
        # ---------------------------------------------------------
        self.critic_mlp = nn.Sequential(
            nn.Linear(critic_obs_dim + latent_dim, 512), nn.ELU(),
            nn.Linear(512, 256), nn.ELU(),
            nn.Linear(256, 1)
        )

        # ---------------------------------------------------------
        # Initialization
        # ---------------------------------------------------------
        self.apply(init_weights)
        nn.init.orthogonal_(self.mean_head_body.weight, gain=0.01)
        nn.init.constant_(self.mean_head_body.bias, 0)
        nn.init.orthogonal_(self.mean_head_hand.weight, gain=0.01)
        nn.init.constant_(self.mean_head_hand.bias, 0)
        nn.init.orthogonal_(self.critic_mlp[-1].weight, gain=1.0)
        nn.init.constant_(self.critic_mlp[-1].bias, 0)

    def forward(self, actor_obs_seq: torch.Tensor, critic_obs_flat: torch.Tensor, z: torch.Tensor) -> Tuple[TransformedDistribution, torch.Tensor]:
        # --- ACTOR PATH ---
        act_ctx = self.actor_history(actor_obs_seq, z)
        actor_features = self.actor_trunk(act_ctx)
        
        # Merge the split heads back into a 39-dim action vector
        mean_body = self.mean_head_body(actor_features)
        mean_hand = self.mean_head_hand(actor_features)
        mean = torch.cat([mean_body, mean_hand], dim=-1)
        
        # Merge the zoned exploration
        log_std = torch.cat([self.log_std_loco, self.log_std_arms, self.log_std_hands])
        
        # Safe float32 boundary + Epsilon
        std = 2.0 * torch.sigmoid(log_std) + 1e-6 
        base_dist = Normal(mean, std)
        dist = TransformedDistribution(base_dist, TanhTransform())

        # --- CRITIC PATH ---
        crit_input = torch.cat([critic_obs_flat, z], dim=-1)
        value = self.critic_mlp(crit_input)
        
        return dist, value

    def get_action(self, actor_obs_seq: torch.Tensor, critic_obs_flat: torch.Tensor, z: torch.Tensor):
        dist, value = self.forward(actor_obs_seq, critic_obs_flat, z)
        action = dist.sample()
        
        # Clamp BEFORE log_prob to prevent -inf ratios
        action = torch.clamp(action, -0.999, 0.999) 
        log_prob = dist.log_prob(action).sum(dim=-1)
        
        return action, log_prob, value.squeeze(-1)

    def evaluate_actions(self, actor_obs_seq: torch.Tensor, critic_obs_flat: torch.Tensor, z: torch.Tensor, actions: torch.Tensor):
        dist, value = self.forward(actor_obs_seq, critic_obs_flat, z)
        
        # Tanh safe-clamp prevents atanh(±1) -> NaN crashes
        actions = torch.clamp(actions, -0.999, 0.999)
        log_prob = dist.log_prob(actions).sum(dim=-1)
        
        # Analytical entropy provides clean PPO gradients
        entropy = dist.base_dist.entropy().sum(dim=-1)
        
        return log_prob, entropy, value.squeeze(-1)

# ========================================================================
# 5. The AMP Discriminator
# ========================================================================
class ASEDiscriminator(nn.Module):
    """Judges the realism of the robot's kinematics against human reference data."""
    def __init__(self, obs_dim: int, latent_dim: int = 16):
        super().__init__()
        self.net = nn.Sequential(
            # Spectral Normalization enforces 1-Lipschitz stability naturally
            nn.utils.spectral_norm(nn.Linear(obs_dim*2 + latent_dim, 512)),
            nn.LeakyReLU(0.2),
            nn.utils.spectral_norm(nn.Linear(512, 512)),
            nn.LeakyReLU(0.2),
            nn.utils.spectral_norm(nn.Linear(512, 1))
        )

    def forward(self, state: torch.Tensor, next_state: torch.Tensor, z: torch.Tensor) -> torch.Tensor:
        # Evaluates the transition context under a specific style
        return self.net(torch.cat([state, next_state, z], dim=-1))