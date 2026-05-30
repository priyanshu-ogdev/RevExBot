# revex_ext/agents/custom_moe_ppo.py
import torch
import torch.nn as nn
from skrl.agents.torch.ppo import PPO

class MoEPPOAgent(PPO):
    def __init__(self, models, memory, cfg, observation_space, action_space, device):
        super().__init__(models, memory, cfg, observation_space, action_space, device)
        
        # 🚨 MOE AUXILIARY LOSS COEFFICIENTS
        self.load_balancing_coef = cfg.get("load_balancing_coef", 0.01)
        self.virtual_opp_coef = cfg.get("virtual_opp_coef", 0.005)

    def _update(self, timestep, timesteps):
        """
        Production-Grade MoE PPO Update.
        Handles base PPO Surrogate + Auxiliary Gating Network Losses.
        """
        # 1. Compute Generalized Advantage Estimation (GAE)
        self.memory.compute_returns_and_advantages(self.models["value"])
        
        # Logging metrics
        cumulative_policy_loss = 0.0
        cumulative_value_loss = 0.0
        cumulative_entropy_loss = 0.0
        cumulative_load_balance_loss = 0.0
        cumulative_virtual_opp_loss = 0.0

        for epoch in range(self.learning_epochs):
            for sampled_batches in self.memory.sample_mini_batches(self.mini_batches):
                
                obs = sampled_batches["states"]
                actions = sampled_batches["actions"]
                advantages = sampled_batches["advantages"]
                returns = sampled_batches["returns"]
                old_log_probs = sampled_batches["log_prob"]

                                with torch.cuda.amp.autocast(enabled=self.mixed_precision):
                    # 🚨 FIXED: Use act() as the single entry point to preserve SKRL state & graph
                    _, _, extra = self.models["policy"].act({"states": obs}, role="policy")
                    
                    # Extract log_probs & entropy from the cached distribution
                    dist = self.models["policy"].get_distribution()
                    curr_log_probs = dist.log_prob(actions).sum(dim=-1, keepdim=True)
                    entropy = dist.entropy().mean()

                    # Evaluate Critic
                    curr_values, _ = self.models["value"].compute({"states": obs}, role="value")

                    # 2. Standard PPO Surrogate Math
                    ratio = torch.exp(curr_log_probs - old_log_probs)
                    surrogate_1 = ratio * advantages
                    surrogate_2 = torch.clamp(ratio, 1.0 - self.clip_predicted_values, 1.0 + self.clip_predicted_values) * advantages
                    policy_loss = -torch.min(surrogate_1, surrogate_2).mean()
                    value_loss = nn.functional.mse_loss(curr_values, returns)
                    entropy_loss = -entropy

                    # 3. 🚨 INTERCEPT AUXILIARY LOSSES FROM THE MoE POLICY
                    aux_lb_loss = torch.tensor(0.0, device=self.device)
                    if "load_balancing_loss" in extra:
                        aux_lb_loss = extra["load_balancing_loss"].mean()
                    
                    aux_vo_loss = torch.tensor(0.0, device=self.device)
                    if "virtual_opponent_loss" in extra:
                        aux_vo_loss = extra["virtual_opponent_loss"].mean()
                        
                    # 4. Total Loss Composite
                    total_loss = (
                        policy_loss 
                        + self.value_loss_scale * value_loss 
                        + self.entropy_loss_scale * entropy_loss 
                        + self.load_balancing_coef * aux_lb_loss
                        + self.virtual_opp_coef * aux_vo_loss
                    )

                # 5. AMP-Safe Backward Pass
                self.optimizer.zero_grad()
                
                if self.mixed_precision:
                    self.scaler.scale(total_loss).backward()
                    self.scaler.unscale_(self.optimizer)
                    nn.utils.clip_grad_norm_(self.models["policy"].parameters(), self.grad_norm_clip)
                    nn.utils.clip_grad_norm_(self.models["value"].parameters(), self.grad_norm_clip)
                    self.scaler.step(self.optimizer)
                    self.scaler.update()
                else:
                    total_loss.backward()
                    nn.utils.clip_grad_norm_(self.models["policy"].parameters(), self.grad_norm_clip)
                    nn.utils.clip_grad_norm_(self.models["value"].parameters(), self.grad_norm_clip)
                    self.optimizer.step()

                # Track metrics
                cumulative_policy_loss += policy_loss.item()
                cumulative_value_loss += value_loss.item()
                cumulative_entropy_loss += entropy_loss.item()
                cumulative_load_balance_loss += aux_lb_loss.item()
                cumulative_virtual_opp_loss += aux_vo_loss.item()

        if self.learning_rate_scheduler is not None:
            self.learning_rate_scheduler.step()

        # Log metrics to SKRL tracker
        steps = self.learning_epochs * self.mini_batches
        self.track_data("Loss / Policy loss", cumulative_policy_loss / steps)
        self.track_data("Loss / Value loss", cumulative_value_loss / steps)
        self.track_data("Loss / Load Balancing loss", cumulative_load_balance_loss / steps)
        self.track_data("Loss / Virtual Opponent loss", cumulative_virtual_opp_loss / steps)