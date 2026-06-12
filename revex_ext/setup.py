"""Installation script for the RevExBot Isaac Lab extension."""

import os
from setuptools import setup, find_packages

# ----------------------------------------------------------------------
#  Production Dependencies
# ----------------------------------------------------------------------
INSTALL_REQUIRES = [
    # ------------------------------------------------------------------
    # Reinforcement Learning & Logging
    # ------------------------------------------------------------------
    "skrl>=1.2.0",                # Isaac Lab tensor‑based env wrapper; widely tested
    "tensorboard>=2.14.0",        # Training telemetry

    # ------------------------------------------------------------------
    # Configuration Management (Isaac Lab internals)
    # ------------------------------------------------------------------
    "hydra-core>=1.3.2",          # Task instantiation
    "omegaconf>=2.3.0",           # YAML runtime parsing
    "pyyaml>=6.0",                # Additional YAML support

    # ------------------------------------------------------------------
    # Data Pipeline – Vision & Kinematics
    # ------------------------------------------------------------------
    "numpy>=1.26.0,<2.0.0",       # STRICT LOCK: NumPy 2.0 breaks Isaac Sim ABI
    "opencv-python>=4.8.0",       # Video I/O, basic image processing
    "scipy>=1.13.0",              # Signal filtering (Savitzky‑Golay), spatial transforms
    "mediapipe>=0.10.14,<0.11",   # Stable 3D holistic pose estimation for Python 3.10
    "ultralytics>=8.3.0",         # YOLOv8‑pose for person detection (lightweight n version)

    # ------------------------------------------------------------------
    # Data Pipeline – Media Harvesting
    # ------------------------------------------------------------------
    "yt-dlp>=2024.12.13",         # CalVer; no API keys needed

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------
    "tqdm>=4.66.0",               # Progress bars
]

# ----------------------------------------------------------------------
#  Optional Extras: VLM pipeline (Qwen2.5‑VL 3B) – fits 48 GB Ada
# ----------------------------------------------------------------------
VLM_REQUIRES = [
    "transformers>=4.46.0",       # HuggingFace transformers with Qwen2.5‑VL support
    "qwen-vl-utils>=0.0.10",      # Official Qwen vision‑language utilities
    "accelerate>=0.33.0",         # Model sharding / device map
    "torch>=2.2.0",               # Already provided by Isaac Lab, but pinned for safety
]

# ----------------------------------------------------------------------
#  Development tools
# ----------------------------------------------------------------------
DEV_REQUIRES = [
    "black>=23.0",
    "isort>=5.12",
    "flake8>=6.0",
]

# ----------------------------------------------------------------------
#  Optional / Development Extras
# ----------------------------------------------------------------------
EXTRAS_REQUIRE = {
    "vlm": VLM_REQUIRES,
    "dev": DEV_REQUIRES,
}

def get_long_description():
    here = os.path.abspath(os.path.dirname(__file__))
    try:
        with open(os.path.join(here, "README.md"), encoding="utf-8") as f:
            return f.read()
    except FileNotFoundError:
        return "Isaac Lab Out-of-Tree Extension for RevExBot Humanoid Training."

setup(
    name="revex_ext",
    version="2.0.0",
    author="RevEx Robotics",
    description="Isaac Lab Out-of-Tree Extension for RevExBot Humanoid Training",
    long_description=get_long_description(),
    long_description_content_type="text/markdown",
    packages=find_packages(include=["revex_ext", "revex_ext.*"]),

    # ------------------------------------------------------------------
    # Non‑Python assets shipped with the package
    # ------------------------------------------------------------------
    package_data={
        "revex_ext": [
            "cfg/env_config.yaml",
            "cfg/train/*.yaml",
            "data/*.json",
        ]
    },

    install_requires=INSTALL_REQUIRES,
    extras_require=EXTRAS_REQUIRE,
    include_package_data=True,

    # 🚨 HARD LOCKED to Python 3.10 for NVIDIA Omniverse / Isaac Lab
    python_requires="~=3.10",

    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Science/Research",
        "Programming Language :: Python :: 3.10",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Operating System :: POSIX :: Linux",
    ],
)