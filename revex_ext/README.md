# RevExBot – Universal Humanoid Motion Brain

A production‑grade reinforcement learning system that trains a **single unified policy** to master locomotion, agility, combat, dance, and precision manipulation on a custom 39‑DOF humanoid robot.

---

## Architecture

- **Unified Environment** – `envs/revex_ase_env_cfg.py` + `envs/revex_ase_env.py`  
  Single, phase‑aware Isaac Lab environment combining all skill domains with dynamic reward modulation.
- **ASE Policy** – `models/ase_policy.py`  
  Split‑head actor‑critic with causal CNN history, variational motion encoder, zoned exploration, and asymmetric critic.
- **Custom Training Loop** – `scripts/train.py`  
  Hybrid SKRL + custom PPO loop with Two‑Time‑Scale Update Rule (TTUR), teleportation masking, dual AMP scalers, and accuracy‑gated discriminator updates.
- **Data Pipeline** – `pipeline/`  
  YouTube harvesting → scene splitting → 3D pose extraction → 39‑DOF retargeting → unified motion library compilation.

---

## Quick Start

### 1. Install Dependencies
```bash
cd RevExBot
pip install -e .
2. Build the Motion Library (Data Pipeline)
bash
cd revex_ext/pipeline
forge.bat
This will:

Search and download ~1000 solo, full‑body videos from YouTube.

Split them into single‑skill clips based on motion pauses.

Extract 3D body + hand landmarks using YOLOv8‑pose and MediaPipe Holistic.

Retarget the 75 landmarks to the RevExBot’s 39‑DOF skeleton.

Compile everything into data/unified_motion_library.json.

3. Phase 1 – Base Locomotion
bash
cd revex_ext/scripts
run.bat 1
Trains a robust walking policy (8192 parallel environments, no style data).
Checkpoints are saved every 500 iterations.

4. Phase 2 – ASE Style Training
bash
run.bat 2 "..\checkpoints\checkpoint_phase1_iter15000.pt"
Adds adversarial style embedding using the motion library and discriminator.
The policy learns to walk, dance, strike, and manipulate with human‑like motion.

5. Play / Export
bash
# Visualise a trained policy
python play.py --phase 2 --checkpoint ..\checkpoints\checkpoint_phase2_iter15000.pt --skill_id combat_jab

# Export to ONNX for edge deployment
python export_onnx.py --checkpoint ..\checkpoints\checkpoint_phase2_iter15000.pt
Key Features
39‑DOF split‑head policy – protects delicate finger gradients from torso‑scale forces.

Causal CNN history – 10‑frame temporal context with strict left‑padding (no future leakage).

Kinetic‑Aware Latent Mixup (KALM) – smooths latent space transitions while preventing kinetic cancellation.

Sim‑to‑Real hardening – stochastic action delay, EKF‑style velocity noise, dynamic impedance morphing, ice‑finger friction, and actuator gain randomisation.

Phase‑aware training – Phase 1 pure locomotion, Phase 2 adversarial style embedding with KL annealing.

Fully vectorised motion library manager – O(1) phase advancement, padded GPU timelines, KNN‑bounded mixup.

Repository Structure
text
revex_ext/
├── assets/               # Robot URDF loader
├── cfg/                  # YAML configs (environment, Phase 1 & 2)
│   └── train/
├── data/                 # Motion library, checkpoints, logs
├── envs/                 # Unified environment + custom MDP
├── models/               # ASE policy & discriminator
├── pipeline/             # Data factory
│   ├── scrape_youtube.py # YouTube harvester
│   ├── ingest_media.py   # Scene splitter
│   ├── extract_kinematics.py # 3D pose extraction
│   ├── retarget_urdf.py  # 39‑DOF retargeting
│   ├── build_library.py  # Motion library compiler
│   ├── motion_library_manager.py # Training‑time manager
│   ├── forge.bat         # Data pipeline orchestrator
│   └── req/              # ffmpeg / ffprobe binaries
└── scripts/              # Training, play, export, batch files
    ├── train.py
    ├── play.py
    ├── export_onnx.py
    └── run.bat
