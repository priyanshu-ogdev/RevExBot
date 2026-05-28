import torch
import torch.nn as nn
from skrl.agents.torch.ppo import PPO

class MoEPPOAgent(PPO):
    def __init__(self, models, memory, cfg, observation_space, action_space, device):
        super().__init__(models, memory, cfg, observation_space, action_space, device)
        # SOTA Load Balancing Coef (Lambda)
        self.load_balancing_coef = cfg.get("load_balancing_coef", 0.01)

    def _update(self, timestep, timesteps):
        """
        16GB-Optimized PPO Update with Auxiliary MoE Loss.
        Iterates over epochs and mini-batches to strictly control VRAM.
        """
        # 1. Compute Generalized Advantage Estimation (GAE)
        self.memory.compute_returns_and_advantages(self.models["value"])
        
        # Logging metrics
        cumulative_policy_loss = 0.0
        cumulative_value_loss = 0.0
        cumulative_entropy_loss = 0.0
        cumulative_aux_loss = 0.0

        # 2. The PPO Epoch Loop
        for epoch in range(self.learning_epochs):
            # 3. The Mini-batch Loop (VRAM Saver)
            for sampled_batches in self.memory.sample_mini_batches(self.mini_batches):
                
                obs = sampled_batches["states"]
                actions = sampled_batches["actions"]
                advantages = sampled_batches["advantages"]
                returns = sampled_batches["returns"]
                old_log_probs = sampled_batches["log_prob"]

                # 4. AMP Context for Forward Pass
                with torch.cuda.amp.autocast(enabled=self.mixed_precision):
                    # Evaluate policy and intercept extra MoE metrics
                    action_mean, action_log_std, extra = self.models["policy"].compute(
                        {"states": obs}, role="policy"
                    )
                    
                    # Reconstruct distribution and current log probs
                    std = torch.exp(action_log_std)
                    dist = torch.distributions.Normal(action_mean, std)
                    curr_log_probs = dist.log_prob(actions).sum(dim=-1, keepdim=True)
                    
                    # Evaluate Critic
                    curr_values, _ = self.models["value"].compute({"states": obs}, role="value")

                    # Standard PPO Surrogate Math
                    ratio = torch.exp(curr_log_probs - old_log_probs)
                    surrogate_1 = ratio * advantages
                    surrogate_2 = torch.clamp(ratio, 1.0 - self.clip_predicted_values, 1.0 + self.clip_predicted_values) * advantages
                    policy_loss = -torch.min(surrogate_1, surrogate_2).mean()

                    value_loss = nn.functional.mse_loss(curr_values, returns)
                    entropy_loss = -dist.entropy().mean()

                    # 🚨 INTERCEPT AUXILIARY LOSS
                    aux_loss = torch.tensor(0.0, device=self.device)
                    if "load_balancing_loss" in extra:
                        aux_loss = extra["load_balancing_loss"].mean()

                    # Total Loss Composite
                    total_loss = (
                        policy_loss 
                        + self.value_loss_scale * value_loss 
                        - self.entropy_loss_scale * entropy_loss 
                        + self.load_balancing_coef * aux_loss
                    )

                # 5. AMP-Safe Backward Pass & Optimization
                self.optimizer.zero_grad()
                
                if self.mixed_precision:
                    self.scaler.scale(total_loss).backward()
                    # Unscale before clipping gradients
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

                # Track metrics for TensorBoard
                cumulative_policy_loss += policy_loss.item()
                cumulative_value_loss += value_loss.item()
                cumulative_entropy_loss += entropy_loss.item()
                cumulative_aux_loss += aux_loss.item()

        # Update Learning Rate Schedulers
        if self.learning_rate_scheduler is not None:
            self.learning_rate_scheduler.step()

        # Log metrics to SKRL tracker
        self.track_data("Loss / Policy loss", cumulative_policy_loss / (self.learning_epochs * self.mini_batches))
        self.track_data("Loss / Value loss", cumulative_value_loss / (self.learning_epochs * self.mini_batches))
        self.track_data("Loss / Load Balancing loss", cumulative_aux_loss / (self.learning_epochs * self.mini_batches))