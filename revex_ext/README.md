# RevExBot: Isaac Lab RL Extension

**Version:** 1.0.0  
**Hardware Target:** Radxa CM5 Edge Compute  
**Framework:** Isaac Lab (Orbit)

This repository contains the Out-of-Tree Isaac Lab extension for the **RevExBot Humanoid**.  
The training pipeline is divided into strict, sequential phases designed to bridge the **Sim-to-Real gap**, focusing on robustness, humanistic compliance, and hardware safety.

---

## 📂 Repository Structure

```
revex_ext/
├── __init__.py                # Package initializer
├── setup.py                   # Python package installer
├── cfg/
│   └── train/
│       ├── phase1_loco_ppo.yaml   # Phase 1: AdamW, High LR, AMP
│       └── phase2_agile_ppo.yaml  # Phase 2: SGD + Momentum, Low LR, AMP
├── envs/
│   ├── __init__.py            # Gymnasium Task Registry
│   ├── custom_mdp.py          # C++ Manager Logic (Slip, Jerk, Symmetry)
│   ├── custom_terrain.py      # Structured Chaos Generator
│   ├── loco/                  # Phase 1: Flat Ground Locomotion
│   │   ├── revex_loco_cfg.py
│   │   └── revex_loco_env.py
│   └── agile/                 # Phase 2: Rough Terrain & Push Recovery
│       ├── revex_agile_cfg.py
│       └── revex_agile_env.py
└── scripts/
    ├── train.py               # Universal RL Runner
    └── play.py                # Visual Evaluator
```

---

## 🛠️ Installation & Setup

1. Open your **Isaac Lab-activated terminal**.  
2. Navigate to the root of this repository.  
3. Install the package in editable mode:

```bash
python -m pip install -e .
```

---

## 🚀 Phase 1: Locomotion Bootstrapping

**Goal:** Teach the robot to balance, walk, and turn on a flat plane while discovering an energy-efficient, humanistic athletic crouch.

### 1. Execute Training
Run the universal train script in headless mode to maximize FPS:

```bash
python scripts/train.py --task RevEx-Loco-v0 --num_envs 2048 --headless
```

### 2. Monitor Success (TensorBoard)
Open a new terminal and launch TensorBoard:

```bash
tensorboard --logdir=runs/
```

**Metrics for Success:**
- `episode_lengths`: Should plateau at 1000.  
- `reward/tracking_lin_vel`: Should trend toward 0.8–1.0.  
- `reward/action_rate_penalty`: Should trend toward 0.  
- `reward/feet_air_time`: Should increase, proving actual steps.

---

## 🌪️ Phase 2: Agile Locomotion & Recovery

**Goal:** Transfer the Phase 1 walking policy to rough terrain, injecting hardware latency, thermal derating, sensor drift, and lateral shoves.

### 1. Execute Training (Load Phase 1 Weights)
Do not train from scratch. Pass the best checkpoint from Phase 1:

```bash
python scripts/train.py --task RevEx-Agile-v0 \
    --checkpoint runs/revex_loco_phase1/nn/revex_loco_phase1.pth \
    --headless
```

### 2. Monitor Success (TensorBoard)
- `loss/policy_loss`: Decreases slowly (SGD + Momentum).  
- `reward/foot_slip`: Must approach 0.  
- `reward/head_stability`: Must approach 0 (critical for VLM integration).

---

## 👁️ Visual Verification (Reality Check)

Graphs can lie; the physics engine cannot. Use the GUI for qualitative evaluation.

### 1. Launch Play Script
```bash
python scripts/play.py --task RevEx-Agile-v0 \
    --checkpoint runs/revex_agile_phase2/nn/revex_agile_phase2.pth
```

### 2. Evaluation Checklist

**❌ Wrong (Over-fitted) Policy:**
- Zombie Walk: Knees locked straight.  
- Tap Dance: Tiny rapid steps.  
- Jitter: High-frequency limb vibration.  
- Stiff Push Recovery: Falls rigidly like a tree.

**✅ Perfect (Humanistic) Policy:**
- Athletic Stance: Slight bend in hips/knees.  
- Contralateral Arm Swing: Natural gait synchronization.  
- Compliant Recovery: Cross-step sideways to regain balance.  
- Head Isolation: Head remains stable for VLM camera feed.

---

## ⏭️ Next Steps

Once Phase 2 verification is flawless, proceed to **Phase 3: Whole-Body Control & VLM Integration**.  
This phase introduces:
- Spatial waypoints  
- Arm manipulation targets  
- RGB camera rendering for multimodal processing  

---

### 📌 Ready for Phase 3
```

