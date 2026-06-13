import os
import json
import torch
import pytest
from revex_ext.pipeline.motion_library_manager import MotionLibraryManager

@pytest.fixture
def dummy_library_path(tmp_path):
    """Creates a temporary, valid unified_motion_library.json for isolated testing."""
    mock_data = {
        "mocap_dim": 39,
        "clips": [
            {
                "skill_id": "test_skill_01",
                "skill_type": "loco",          # ← required
                "duration_s": 1.0,            # ← required
                "latent_z": [0.0] * 16,       # ← required (16‑dim latent)
                "frames": [
                    {
                        "joint_pos": [0.0] * 39,
                        "joint_vel": [0.0] * 39,
                        "contact_schedule": [0, 0, 0, 0],
                        "stiffness_mult": 1.0,
                        "rhythm_beat": 0.0
                    },
                    {
                        "joint_pos": [0.1] * 39,
                        "joint_vel": [0.0] * 39,
                        "contact_schedule": [0, 0, 0, 0],
                        "stiffness_mult": 1.0,
                        "rhythm_beat": 0.0
                    }
                ]
            }
        ]
    }
    file_path = tmp_path / "test_library.json"
    with open(file_path, "w") as f:
        json.dump(mock_data, f)
    return str(file_path)

def test_manager_loads_and_samples(dummy_library_path):
    """Ensure the manager loads the JSON schema and samples transitions cleanly on CPU."""
    mgr = MotionLibraryManager(dummy_library_path, device="cpu")
    
    assert mgr.num_clips == 1, "Failed to parse mock clip data."
    assert not torch.isnan(mgr.padded_joint_pos).any(), "Parsed coordinates contain NaNs."
    
    # Test tensor generation and sampling mechanics
    s, s_next, z = mgr.sample_real_transitions(batch_size=2, device="cpu")
    assert s.shape == (2, 39), "Sampled state space dimension mismatch."
    assert not torch.isnan(s).any(), "Manager sampled NaN states."