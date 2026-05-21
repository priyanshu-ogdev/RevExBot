"""
Root module for RevExBot Extension.
"""
from omni.isaac.lab_tasks.utils import import_packages

# Import all environments to trigger Gymnasium registration
import_packages(__name__, ["envs"])