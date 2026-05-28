"""Installation script for the RevExBot Isaac Lab extension."""

import os
from setuptools import setup, find_packages

# 🚨 PRODUCTION DEPENDENCIES
# Isaac Lab natively provides PyTorch and PhysX. 
# We only list the external libraries required for our custom MoE pipeline.
INSTALL_REQUIRES = [
    # RL Backends (LTS Versions)
    "rl-games>=1.6.0",         # For Phase 1 & Phase 2 baselines
    "skrl>=1.2.0",             # For Phase 3 Mixture of Experts (MoE)
    "tensorboard>=2.14.0",     # For logging metrics
    
    # Configuration Management
    "hydra-core>=1.3.2",       # For Isaac Lab task instantiation
    "omegaconf>=2.3.0",        # YAML parsing
    
    # Kinematic Retargeting & Vision Daemon Dependencies
    "numpy>=1.23.0",           # For .npy AMP motion parsing
    "opencv-python>=4.8.0",    # For video ingestion processing
    "mediapipe>=0.10.0",       # (Optional) Standard for human pose estimation
    
    # Utilities
    "h5py>=3.9.0",             # Often required for large dataset handling
]

# Function to read the README for the long description
def get_long_description():
    here = os.path.abspath(os.path.dirname(__file__))
    try:
        with open(os.path.join(here, "README.md"), encoding="utf-8") as f:
            return f.read()
    except FileNotFoundError:
        return "Isaac Lab Out-of-Tree Extension for RevExBot Humanoid Training."

setup(
    name="revex_ext",
    version="1.0.0",
    author="RevEx Robotics",
    description="Isaac Lab Out-of-Tree Extension for RevExBot Humanoid Training",
    long_description=get_long_description(),
    long_description_content_type="text/markdown",
    packages=find_packages(include=["revex_ext", "revex_ext.*"]),
    install_requires=INSTALL_REQUIRES,
    include_package_data=True,
    python_requires=">=3.10", # Isaac Lab standard
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Science/Research",
        "Programming Language :: Python :: 3.10",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
    ],
)