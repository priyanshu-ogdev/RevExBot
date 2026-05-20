
# RevExBot – Production Build

This repository contains everything needed to train and deploy a full‑body humanoid robot (codename **RevExBot**) with integrated dexterous hands.

The directory structure is designed for a **three‑phase workflow**:

| Phase | Activity | Status |
|-------|----------|--------|
| 1 | Hardware design & simulation modelling | ✅ Complete |
| 2 | Reinforcement learning training (Isaac Lab) | ⏳ Next |
| 3 | Physical fabrication, assembly & deployment | 📦 Files ready |

---

## 📁 Top‑Level Folders
assets/ – Visual & collision meshes, URDF/Xacro definitions
build/ – Build scripts (URDF compilation, generation)
electrical/ – KiCad PCBs, wiring diagrams, firmware overlays
isaac_lab_config/ – Environment YAML for Isaac Lab
mechanical/ – CAD fabrication files (STEP), manifests, mechanical meshes
output/ – Final compiled URDF
rl_hands/ – RL training environments, policy exporters, task logic



---

## 🎨 `assets/`

Contains the simulation‑ready meshes and the robot’s kinematic description.

| Subfolder | Contents |
|-----------|----------|
| `assets/meshes/` | `.STL` files for every body link (hips, knees, ankles, shoulders, elbows, wrists, neck, waist, toes) **and** both hands (`lh_*`, `rh_*`). All meshes have been verified to align with the MuJoCo/Isaac Lab coordinate conventions. |
| `assets/xacro/` | `faive_hand.xacro` and `master_assembly.xacro` – the master URDF macro files that assemble the complete robot. The hand xacro is production‑ready with correct joint axes, limits, collision primitives, and contact exclusion groups. |

**Phase 2 usage:** Loaded directly by Isaac Lab’s asset importer.  
**Phase 3 usage:** The same meshes serve as visual reference for physical assembly.

---

## ⚙️ `build/`

Helper scripts for code generation and asset conversion.

| File | Purpose |
|------|---------|
| `compile_urdf.py` | Converts the `.xacro` files into a flat `.urdf` (`output/revexbot.urdf`). |
| `convert_to usd.bat` | Batch file to convert `.stl`/`.urdf` to USD for NVIDIA Omniverse. |
| `generate_fabrication.py` | Automatically produces the fabrication manifest from the STEP file tree. |

**Phase 2:** Run `compile_urdf.py` before starting training if the xacro changes.  
**Phase 3:** `generate_fabrication.py` can be re‑run to update BOMs if the design evolves.

---

## ⚡ `electrical/`

All electrical design assets for the physical robot.
electrical/
├── README.md
├── motion_control/
│ ├── mcb-io.dts # Device tree overlay (Radxa CM5 SPI/CAN)
│ ├── board/ # KiCad project – carrier board for Radxa CM5
│ │ ├── board.kicad_pro
│ │ ├── board.kicad_pcb / .sch / .dru
│ │ ├── can.kicad_sch, spi-can.kicad_sch, …
│ │ ├── fp-lib-table, sym-lib-table
│ │ ├── lcsc_lib.3dshapes/ # 3D models for PCB preview
│ │ ├── lcsc_lib.pretty/ # LCSC component footprints
│ │ └── radxa.pretty/ # Radxa CM5 footprint
│ └── scripts/
│ └── serial.sh # Serial port init script
└── wiring/
├── wiring.svg # Wiring harness diagram
└── wiring.yaml # Harness mapping (pin‑to‑pin)



**Phase 2:** Not used – simulation runs entirely in software.  
**Phase 3:**
- Send Gerber files (generated from `board/`) to a PCB fab (JLCPCB/PCBWay).
- Flash `mcb-io.dts` and run `serial.sh` on the Radxa CM5.
- Use `wiring.svg` and `wiring.yaml` to connect motors, encoders, and CAN bus.

---

## 🧪 `isaac_lab_config/`

Holds the environment definition for Isaac Lab.

| File | Purpose |
|------|---------|
| `revexbot_env.yaml` | Configuration for the training environment (simulator settings, robot asset path, domain randomisation, etc.). |

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

## 📋 Phase 2 Quickstart (RL Training)

1. Install Isaac Lab (NVIDIA Omniverse + Isaac Sim + Isaac Lab).
2. Set the environment variable `ISAAC_LAB_PATH`.
3. Run `python rl_hands/train.py` with the desired task config.
4. Monitor training with TensorBoard.

---

## 🔧 Phase 3 Quickstart (Physical Build)

1. **Mechanical:**  
   - Zip `mechanical/RVX1/*/FABRICATION/ALU_7075` → send to CNC.  
   - Zip `MJF_PA12` → send to MJF printing.  
   - Zip `SML_316L` → send to SLM printing.  
   - Order OTS parts according to `FABRICATION_MANIFEST.csv`.

2. **Electrical:**  
   - Generate Gerber & drill files from `electrical/motion_control/board/` → send to PCB fab.  
   - Populate PCB using the interactive BOM.  
   - Wire motors and sensors according to `wiring.yaml`.

3. **Software:**  
   - Export trained policy to ONNX.  
   - Deploy ONNX model + inference loop on Radxa CM5.

---

## 📝 Notes

- All CAD files have been renamed from `ASV1` → `RVX1` to reflect the new project name.
- The `rl_hands/` folder is a cleaned‑up version of the original `faive_gym_oss` repository, containing only the essential training and deployment scripts.

---

*Last updated: 2026‑05‑20*