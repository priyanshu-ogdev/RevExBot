import numpy as np
from revex_ext.pipeline.retarget_urdf import extract_full_limb_kinematics

def test_tpose_body_angles_zero():
    """A perfect T‑pose should yield near‑zero joint angles."""
    # shoulder, elbow, wrist (straight line in front of body)
    shoulder = np.array([0.2, 0.0, 0.0])
    elbow   = np.array([0.4, 0.0, 0.0])
    wrist   = np.array([0.6, 0.0, 0.0])
    finger  = np.array([0.7, 0.0, 0.0])
    R_parent_inv = np.eye(3)

    angles = extract_full_limb_kinematics(shoulder, elbow, wrist, finger, R_parent_inv, is_left_side=False)
    # pitch, roll, yaw, elbow_angle, wrist_flex, toe (unused)
    assert abs(angles[0]) < 0.1    # shoulder pitch ≈ 0
    assert abs(angles[3]) < 0.1    # elbow angle ≈ 0 (arm straight)
    assert abs(angles[4]) < 0.1    # wrist flex ≈ 0