"""Installation script for the RevExBot Isaac Lab extension."""

import os
from setuptools import setup, find_packages

# Read the requirements if you have any custom ones (optional for now)
INSTALL_REQUIRES = [
    # "omni.isaac.lab" is assumed to be installed via the Isaac Sim environment
]

setup(
    name="revex_ext",
    version="1.0.0",
    author="RevEx Robotics",
    description="Isaac Lab Out-of-Tree Extension for RevExBot Humanoid Training",
    packages=find_packages(),
    install_requires=INSTALL_REQUIRES,
    include_package_data=True,
    python_requires=">=3.10",
)