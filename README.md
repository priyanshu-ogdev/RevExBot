
<div align="center">

# 🤖 RevExBot – Humanoid Robot Production Infrastructure

[![Apache License 2.0](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-Production%20Ready-brightgreen.svg)]()
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)]()
[![NVIDIA Isaac Lab](https://img.shields.io/badge/isaac_lab-powered-76B900.svg)]()

A complete, open-source infrastructure for designing, training, and deploying a full‑body humanoid robot with integrated dexterous hands. From CAD to reinforcement learning to physical fabrication.

[Documentation](#-documentation) • [Quick Start](#-quick-start) • [Architecture](#-architecture) • [Contributing](#-contributing)

</div>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Architecture](#-architecture)
- [Repository Structure](#-repository-structure)
- [Documentation](#-documentation)
- [Quick Start](#-quick-start)
- [Phase Workflows](#-phase-workflows)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🎯 Overview

**RevExBot** is a production-grade infrastructure for autonomous humanoid robot development, implementing a **three-phase workflow** that bridges simulation, training, and physical deployment:

| Phase | Focus | Status | Timeline |
|-------|-------|--------|----------|
| **Phase 1** | Hardware design & simulation modelling | ✅ Complete | Foundation |
| **Phase 2** | Reinforcement learning training (Isaac Lab) | ⏳ In Progress | Training |
| **Phase 3** | Physical fabrication, assembly & deployment | 📦 Ready | Production |

This repository is fully self-contained with CAD files, electrical schematics, trained policies, and deployment scripts for edge compute (Radxa CM5).

---

## 🏗️ Architecture

```
RevExBot Infrastructure
├── assets/               CAD, meshes, URDF/Xacro kinematic models
├── hardware/
│   ├── electrical/       KiCad PCB design, wiring harnesses
│   └── mechanical/       STEP files, fabrication manifests
├── revex_ext/            Isaac Lab extension (RL training pipeline)
└── output/               Compiled URDF and deployment artifacts
```

**Key Technologies:**
- 🎮 **Isaac Lab** – Physics simulation and RL training
- 🧠 **RL-Games / SKRL** – Multi-domain expert training
- 🔧 **KiCad** – PCB design and electrical control
- 🤖 **URDF/Xacro** – Kinematic robot description
- 🚀 **Radxa CM5** – Edge compute for 200Hz control loop

---

## 📁 Repository Structure
### Top-Level Directories

| Directory | Purpose | Phase |
|-----------|---------|-------|
| **`assets/`** | Visual & collision meshes, URDF/Xacro kinematic models | 1, 2, 3 |
| **`build/`** | Build automation scripts (URDF compilation, format conversion) | 2, 3 |
| **`hardware/`** | Electrical (KiCad PCB) and mechanical (CAD/STEP) designs | 3 |
| **`isaac_lab_config/`** | Isaac Lab environment configurations | 2 |
| **`output/`** | Compiled URDF and deployment-ready artifacts | 2, 3 |
| **`revex_ext/`** | Isaac Lab extension with full RL training pipeline | 2 |

---

### 📂 Directory Deep-Dive

#### 🎨 `assets/` – Kinematic Models & Meshes

Contains simulation-ready meshes and the robot's complete kinematic description.

| Subfolder | Contents | Use Case |
|-----------|----------|----------|
| `meshes/` | `.STL` collision and visual geometry for all body links and hands | Simulation, physics, 3D rendering |
| `xacro/` | `master_assembly.xacro`, `faive_hand.xacro` – production URDF macros | Kinematic chain definition, joint configuration |
| `mjcf/` | MuJoCo XML format (alternative to URDF) | Direct MuJoCo simulation |
| `urdf/` | Legacy URDF files (kept for reference) | Backward compatibility |

**Phase 2 Usage:** Isaac Lab imports these directly via the asset importer.  
**Phase 3 Usage:** Same meshes serve as visual reference for physical assembly.

---

## 🔨 `build/` – Code Generation & Automation

Helper scripts for asset compilation and format conversion.

| Script | Purpose | Output |
|--------|---------|--------|
| `compile_urdf.py` | Converts Xacro → flat URDF | `output/revexbot.urdf` |
| `convert_to_usd.bat` | Batch format conversion for Omniverse | USD native format |
| `generate_fabrication.py` | Auto-generates BOM from STEP file tree | `FABRICATION_MANIFEST.json` |

**When to run:**
- Before Phase 2: Any changes to xacro files
- Before Phase 3: Fabrication manifest updates

---

## ⚡ `hardware/` – Electrical & Mechanical Design

All electrical design assets for the physical robot.
electrical/
├── README.md
├── motion_control/
│   ├── mcb-io.dts # Device tree overlay (Radxa CM5 SPI/CAN)
│   ├── board/ # KiCad project – carrier board for Radxa CM5
│   └── scripts/
│       └── serial.sh # Serial port init script
└── wiring/
    ├── wiring.svg # Wiring harness diagram
    └── wiring.yaml # Harness mapping (pin-to-pin)

**Electrical** (`hardware/electrical/`)  
- `motion_control/board/` – Full KiCad PCB project for Radxa CM5 carrier board
- `motion_control/scripts/` – Linux device tree overlays and init scripts
- `wiring/` – Harness documentation and pin-to-pin mappings

**Mechanical** (`hardware/mechanical/`)  
- `RVX1/` – Master CAD assembly and sub-assemblies (grouped 100–700)
- `FABRICATION/` – Per-material fabrication packages (CNC, MJF, SLM, OTS)
- `FABRICATION_MANIFEST.json` – Complete BOM with part numbers and vendors

**Phase 2:** Not used (simulation only).  
**Phase 3:** Primary artifact for fabrication workflow.

---

## 🧠 `revex_ext/` – Isaac Lab RL Training Extension

The heart of Phase 2 training. Full out-of-tree extension with:
- **Environments:** Locomotion, agile recovery, tri-domain experts (combat/dance/precision)
- **Training scripts:** Multi-phase RL pipeline with expert/router/finetune modes
- **Policy export:** ONNX conversion for edge deployment

[See `revex_ext/README.md`](revex_ext/README.md) for detailed training workflows.

---

## 📤 `output/` – Compiled Deployment Artifacts

| File | Purpose | Generated By |
|------|---------|--------------|
| `revexbot.urdf` | Flat URDF for Isaac Lab / MuJoCo | `build/compile_urdf.py` |



**Phase 2:** Not used – simulation runs entirely in software.  
**Phase 3:**
- Send Gerber files (generated from `board/`) to a PCB fab (JLCPCB/PCBWay).
- Flash `mcb-io.dts` and run `serial.sh` on the Radxa CM5.
- Use `wiring.svg` and `wiring.yaml` to connect motors, encoders, and CAN bus.

---

## 📖 Documentation

### Core Guides

| Document | Purpose | Audience |
|----------|---------|----------|
| [revex_ext/README.md](revex_ext/README.md) | Full RL training & MoE architecture | ML Engineers |
| [REFERENCES.md](REFERENCES.md) | Research papers & citations | Researchers |
### Asset Reference

- **Kinematic chain:** [`assets/xacro/master_assembly.xacro`](assets/xacro/master_assembly.xacro)
- **Hand design:** [`assets/xacro/faive_hand.xacro`](assets/xacro/faive_hand.xacro)
- **Fabrication manifest:** [`hardware/mechanical/FABRICATION_MANIFEST.json`](hardware/mechanical/FABRICATION_MANIFEST.json)

---

## 🚀 Quick Start

### Prerequisites

- **Phase 2 (RL Training):**
  - NVIDIA GPU (RTX 4070 Ti or better)
  - NVIDIA Isaac Lab + Isaac Sim
  - Python 3.10+
  - PyTorch, SKRL, RL-Games

- **Phase 3 (Hardware Assembly):**
  - KiCad 8.0+ (for PCB design verification)
  - CAM software for STEP file visualization
  - Fabrication partner access (CNC, MJF, SLM)

### Phase 2: RL Training

```bash
# 1. Compile URDF from Xacro
python build/compile_urdf.py

# 2. Verify Isaac Lab installation
echo $ISAAC_LAB_PATH

# 3. Start training (see revex_ext/README.md for full options)
cd revex_ext
python scripts/train.py --task RevEx-Loco-v0 --phase expert --headless

# 4. Monitor with TensorBoard
tensorboard --logdir runs/
```

### Phase 3: Physical Fabrication

```bash
# 1. Generate/update fabrication manifest
python build/generate_fabrication.py

# 2. Review BOM
cat hardware/mechanical/FABRICATION_MANIFEST.csv

# 3. Organize by material and vendor
# - ALU_7075/ → CNC machining
# - MJF_PA12/ → Multi Jet Fusion 3D printing
# - SML_316L/ → SLM metal printing
# - OFF_THE_SHELF/ → Standard components

# 4. Electrical: Send to PCB fab
# - Navigate to: hardware/electrical/motion_control/board/
# - Generate Gerber files in KiCad
# - Upload to JLCPCB, PCBWay, or local fab
```

**Phase 2:** This file is the entry point for Isaac Lab. Modify it to adjust the simulation before launching training.

---

## 🛠️ `mechanical/`

The **physical fabrication vault** – every part needed to build the robot’s body.
mechanical/
├── FABRICATION_MANIFEST.csv / .json # Bill of Materials: part numbers, material, quantity, manufacturer
├── naming_convention.png # Visual key for part numbering scheme
├── meshes/
│ └── leap_hand/ # (Optional) Leap hand reference meshes
└── RVX1/ # CAD assembly (formerly ASV1)
├── REVEXBOT_V1.STEP # Master assembly STEP
├── 100/ … 700/ # Sub‑assembly groupings (hips, knees, etc.)
│ ├── RVX1_XXX.STEP # Assembly STEP for each sub‑group
│ └── FABRICATION/
│ ├── ALU_7075/ # CNC aluminium parts
│ ├── MJF_PA12/ # 3D‑printed nylon parts (Multi Jet Fusion)
│ ├── SML_316L/ # Stainless steel parts
│ └── OFF_THE_SHELF/ # Standard mechanical components (bearings, pulleys)



**Phase 2:** Not used directly – simulation uses `assets/meshes/`.  
**Phase 3:**
1. Read `FABRICATION_MANIFEST.csv` to understand material, quantity, and vendor info.
2. Zip each material subfolder and upload to the corresponding manufacturer:
   - `ALU_7075` → CNC machining service
   - `MJF_PA12` → MJF 3D printing service
   - `SML_316L` → Metal 3D printing (SLM)
   - `OFF_THE_SHELF` → Order from listed suppliers (bearings, fasteners, etc.)
3. The folder and file names use **RVX1** as the project prefix (formerly ASV1). All parts are readily identifiable.

---

## 📤 `output/`

Final compiled robot definitions.

| File | Purpose |
|------|---------|
| `revexbot.urdf` | Flat URDF of the entire robot, generated from the `.xacro` files. |

**Phase 2:** Load this URDF in Isaac Lab (or use the MuJoCo‑native MJCF from `assets/xacro/`).  
**Phase 3:** Not needed for hardware; only for on‑robot inverse kinematics if deploying a full controller.

---

## 🧠 `rl_hands/`

Everything needed to train and deploy dexterous manipulation policies.
rl_hands/
├── robot_hand.py # Gym environment definition (action/observation spaces, reward)
├── train.py # PPO training script (launches Isaac Lab)
├── setup.py # Package setup
├── init.py
├── cfg/
│ ├── config.yaml # Global hyperparameters
│ ├── task/ # Per‑task environment configs (crawl, sphere, book, …)
│ └── train/ # PPO‑specific configs for each task
├── scripts/
│ ├── export_policy.py # Export a single trained checkpoint to ONNX
│ └── export_all_policies.py # Batch export all checkpoints
└── tasks/
└── crawl.py # Crawling task logic (mapping actions to joint targets)



**Phase 2:**  
1. Edit `cfg/task/FaiveHandP0_crawl.yaml` (or other tasks) to match your robot’s action space.  
2. Run `python train.py` to start training.  
3. Checkpoints are saved inside Isaac Lab’s log directory.

**Phase 3:**  
- Use `export_policy.py` to convert the best checkpoint to ONNX.  
- Deploy the ONNX model on the Radxa CM5 using your inference runtime (TensorRT / ONNX Runtime).

---

## � Phase Workflows

### ✅ Phase 1: Design & Simulation (Complete)

- CAD models finalized in Fusion 360 / native CAD
- URDF/Xacro kinematic models validated
- Collision meshes optimized for simulation
- Ready for Isaac Lab import

**Next Step:** → Phase 2

---

### ⏳ Phase 2: RL Training (In Progress)

**Goal:** Train multi-domain expert policies using Isaac Lab

**Workflow:**
1. **Base Locomotion** – Learn flat-ground walking
   ```bash
   python revex_ext/scripts/train.py --task RevEx-Loco-v0 --phase expert
   ```

2. **Agile Recovery** – Transfer to rough terrain with perturbations
   ```bash
   python revex_ext/scripts/train.py --task RevEx-Agile-v0 --phase expert
   ```

3. **Tri-Domain Experts** – Train specialized skill policies
   - Combat (explosive dynamics)
   - Dance (momentum dissipation)
   - Precision (tactile manipulation)
   ```bash
   python revex_ext/scripts/train.py --task RevEx-Combat-v0 --phase expert
   ```

4. **MoE Router** – Train gating network
   ```bash
   python revex_ext/scripts/train.py --task RevEx-Combat-v0 --phase router
   python revex_ext/scripts/train.py --task RevEx-Combat-v0 --phase finetune
   ```

5. **Export to ONNX**
   ```bash
   python revex_ext/scripts/export_moe_onnx.py
   ```

**Deliverables:**
- Trained policy checkpoints
- ONNX graph for edge deployment
- Validation videos

**Next Step:** → Phase 3

---

### 📦 Phase 3: Fabrication & Deployment (Ready)

**Goal:** Build physical robot and deploy trained policies

**Workflow:**

#### A. Mechanical Fabrication
1. Download fabrication packages from `hardware/mechanical/RVX1/*/FABRICATION/`
2. Send each material folder to appropriate manufacturer:
   - **ALU_7075/** → CNC machining service (e.g., Xometry, SendCutSend)
   - **MJF_PA12/** → Multi Jet Fusion service (e.g., HP, EOS)
   - **SML_316L/** → Stainless steel SLM printing
   - **OFF_THE_SHELF/** → Order from BOM suppliers
3. Track parts and receipt in spreadsheet
4. Organize assembly station

#### B. Electrical Assembly
1. Review PCB design in `hardware/electrical/motion_control/board/`
2. Generate Gerber files and upload to fab house
3. Wait for PCB population
4. Flash Radxa CM5 with Linux + device tree overlay (`mcb-io.dts`)
5. Test CAN bus communication per `wiring.yaml`

#### C. Robot Assembly
1. Follow mechanical assembly drawings (STEP files as reference)
2. Sub-assembly groups (100 → hips, 200 → knees, etc.)
3. Integrate electronics per `hardware/wiring/wiring.yaml`
4. Perform cable management and strain relief

#### D. Software Deployment
1. Copy ONNX model to Radxa CM5
2. Deploy inference runtime (TensorRT / ONNX Runtime)
3. Validate 200Hz control loop latency
4. Calibrate sensors and motor parameters

**Deliverables:**
- Fully assembled robot
- Functional 200Hz control loop
- Deployed MoE policies running on edge hardware

---

## 🤝 Contributing

We welcome contributions across all domains:

### For Machine Learning Researchers
- Improve policy training architectures
- Add new task domains
- Enhance reward shaping
- Submit PRs to `revex_ext/`

### For Hardware Engineers
- CAD improvements and optimizations
- PCB layout refinements
- Mechanical part alternatives
- Submit PRs to `hardware/`

### For Roboticists
- Deployment optimizations for edge hardware
- Control loop tuning
- Sensor calibration scripts
- Submit PRs to root directory

### Guidelines

1. **Fork** the repository
2. **Create a feature branch** (`feature/your-contribution`)
3. **Test thoroughly** before submitting
4. **Submit a Pull Request** with detailed description
5. **Link any relevant issues** or discussions

### Code of Conduct

Please be respectful and constructive in all interactions.

---

## � License

This project is licensed under the **Apache License 2.0** – see the [LICENSE](LICENSE) file for full details.

### Summary

- ✅ Free to use, modify, and distribute
- ✅ Must include license notice
- ✅ State changes made to the code
- ✅ Include original copyright notice

### DISCLAIMER

This is research and experimental software. While we've made efforts to ensure stability and safety, use in production environments is at your own risk. Always conduct thorough testing in simulation before deploying to hardware.

---

## 📚 References & Citations

For the research foundations of this work, see [REFERENCES.md](REFERENCES.md).

---

## 🙋 Support & Contact

- **Issues & Bugs:** [GitHub Issues](../../issues)
- **Discussions:** [GitHub Discussions](../../discussions)
- **Documentation:** Check relevant READMEs in each subfolder

---

## 🎉 Acknowledgments

RevExBot builds on years of research in humanoid robotics, reinforcement learning, and hardware engineering. We thank all contributors and the open-source community.

---

<div align="center">

**Made with ❤️ for the robotics community**

*Last updated: May 2026*

</div>