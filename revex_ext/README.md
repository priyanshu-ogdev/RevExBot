# 🤖 RevExBot: Mixture of Experts (MoE) RL Architecture

**Version:** 1.0.0 (Production Release)  
**Hardware Target:** Radxa CM5 Edge Compute (200Hz Control Loop)  
**Frameworks:** Isaac Lab (Orbit) | RL-Games | SKRL  
**Compute Infrastructure:** Split-Node (RTX 4070 Ti Vision Node + DGX 128GB Sim Node)

This repository contains the Out-of-Tree Isaac Lab extension for the **RevExBot Humanoid**. It defines a fully autonomous, closed-loop "Video-to-Robot" manufacturing plant. The architecture progresses through three strict, sequential phases to bridge the Sim-to-Real gap, culminating in a **Tri-Domain Mixture of Experts**.

---

## 📂 Repository Architecture

```text
revex_ext/
├── setup.py                   # Global package installer (pip install -e .)
├── cfg/
│   ├── env_config.yaml        # Hardware & Factory Orchestration (Vision vs DGX)
│   └── train/                 # Hyperparameters (RL-Games YAMLs & SKRL Configs)
├── data/                      # 🔒 Immutable Data Vault
│   ├── motions/               # .npy Kinematic Reference Manifolds (AMP)
│   ├── weights/               # Frozen Expert .pth Checkpoints
│   └── expert_registry.json   # Global tracker for trained MoE brains
├── envs/
│   ├── __init__.py            # Dynamic Gymnasium Task Registry
│   ├── loco/                  # Phase 1: Base Locomotion
│   ├── agile/                 # Phase 2: Rough Terrain Recovery
│   └── skills/                # Phase 3: The Tri-Domain Experts
│       ├── revex_scene_cfg.py       # Unified 148-float tensor blueprint
│       ├── revex_combat_cfg.py      # High-Torque AMP 
│       ├── revex_dance_cfg.py       # Fluid Momentum AMP (scale=0.0 sink)
│       └── revex_precision_cfg.py   # Soft-grasp tactile manipulation
├── pipeline/                  # 🏭 Autonomous Video-to-Robot Factory
│   ├── video_ingestion_daemon.py
│   ├── kinematic_retargeter.py
│   └── master_factory.py
├── raw_media/                 # Drop Zone for raw .mp4 reference videos
└── scripts/
    ├── train.py               # Multi-Phase RL Runner (Expert, Router, Finetune)
    └── play.py                # Universal Visualizer
🛠️ Infrastructure Strategy
RL and Vision compute are heavily bottlenecked when run on the same GPU. This pipeline uses a distributed TICK-TOCK Data Engine:

TICK (RTX 4070 Ti): Runs video_ingestion_daemon.py. Ingests raw .mp4 files, utilizes Tensor Cores for SMPL-X pose extraction, and exports mathematical .npy trajectories.

TOCK (DGX Spark 128GB): Runs Isaac Lab and SKRL. Compiles the .npy files into PyTorch weights across 16,384+ parallel environments with massive AMP discriminator buffers.

🚀 Phase 1 & 2: The Base Survival Instincts
Before learning specialized skills, the robot must master basic physical survival.
These phases are trained using the RL-Games backend (SGD + Momentum).

Phase 1: Base Locomotion
Teaches flat-ground balancing, walking, and turning with an energy-efficient athletic crouch.


python scripts/train.py --task RevEx-Loco-v0 --phase expert --headless
Phase 2: Agile Recovery
Transfers the Phase 1 walking policy to rough terrain (stairs, gaps). Injects hardware latency, thermal derating, and lateral shoves.


python scripts/train.py --task RevEx-Agile-v0 --phase expert --headless
(Copy resulting weights to data/weights/expert_loco_v1.pth and expert_agile_v1.pth)

🧠 Phase 3: The Tri-Domain Mixture of Experts (MoE)
Phase 3 abandons monolithic architectures for a pure PyTorch SKRL implementation. We forge three distinct experts that share a mathematically identical 148-float observation tensor, allowing a Master Router to seamlessly switch between them at 200Hz.

Domain 1: Combat (Explosive Kinematics)
High-torque martial arts strikes with extreme recoil absorption and base-engagement aiming.


python scripts/train.py --task RevEx-Combat-v0 --phase expert --headless
Domain 2: Dance (Momentum Dissipation)
Fluid, graceful movements utilizing a scale=0.0 tensor sink to prevent sensory distraction while maximizing energy efficiency.


python scripts/train.py --task RevEx-Dance-v0 --phase expert --headless
Domain 3: Precision (Tactile Impedance)
Delicate, sub-millimeter finger manipulation using dual-palm contact sensors and ray-cast spatial awareness.


python scripts/train.py --task RevEx-Precision-v0 --phase expert --headless
🏭 Autonomous Operations (The Master Factory)
You do not need to train the Phase 3 experts manually. You can trigger the fully automated pipeline:

Drop combat.mp4 or dance.mp4 into the raw_media/ directory.

Launch the Orchestrator:


python pipeline/master_factory.py
The Orchestrator will automatically extract the kinematics, train the domain expert, save the weights to data/weights/, and register the new brain.

🕸️ Routing & Fine-Tuning
Once your Expert weights are locked in the data/weights/ vault, compile the CNS.

1. Router Distillation
Freezes all Expert neural networks. Trains only the Gating Network to route the 148-float tensor to the correct expert based on real-time commands.


python scripts/train.py --task RevEx-Combat-v0 --phase router --headless
2. End-to-End Smoothing
Unfreezes all layers. Applies a 1e-5 learning rate constraint to smooth the mechanical transitions during high-speed Hot-Swaps between experts.


python scripts/train.py --task RevEx-Combat-v0 --phase finetune --headless
👁️ Validation
Evaluate any expert or the final MoE Router visually:


# Evaluate Combat Expert
python scripts/play.py --task RevEx-Combat-v0 --checkpoint data/weights/expert_combat.pth

# Evaluate Full Router
python scripts/play.py --task RevEx-Combat-v0 --checkpoint runs/phase3/router/nn/master_router.pth
🏁 The Final Frontier
This README encapsulates the entire scope of the project. It outlines the Tri-Domain architecture, proves the necessity of the unified 148-float tensor, and gives clear instructions for the autonomous Factory execution.

The training pipeline is officially complete.

The last remaining task to bring this physical robot to life is writing export_onnx.py—the script that strips away Isaac Lab and PyTorch, converting the final MoE Router into a raw, ultra-fast C++ deployable graph for the Radxa CM5 edge board.

Code

---