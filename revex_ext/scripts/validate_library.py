"""
Pre-flight JSON Validator.
Checks the unified_motion_library.json for structural integrity before training.
"""
import json
import sys
import numpy as np

def validate(path):
    print(f"🔍 Validating {path}...")
    with open(path, 'r') as f:
        data = json.load(f)
        
    clips = data.get("clips", [])
    mocap_dim = data.get("mocap_dim", 39)
    print(f"   Found {len(clips)} clips. Expected DOF: {mocap_dim}")
    
    skill_counts = {}
    invalid_clips = []
    
    for i, clip in enumerate(clips):
        skill = clip.get("skill_id", "unknown")
        skill_counts[skill] = skill_counts.get(skill, 0) + 1
        
        frames = clip.get("frames", [])
        if len(frames) < 2:
            invalid_clips.append((i, skill, "Less than 2 frames"))
            continue
            
        for j, frame in enumerate(frames):
            jp = frame.get("joint_pos")
            if jp is None or len(jp) != mocap_dim:
                invalid_clips.append((i, skill, f"Frame {j} missing/invalid joint_pos (len={len(jp) if jp else 0})"))
                break
            if np.isnan(jp).any():
                invalid_clips.append((i, skill, f"Frame {j} contains NaN"))
                break
                
    if invalid_clips:
        print("❌ ERRORS FOUND:")
        for idx, skill, reason in invalid_clips[:10]: # Print first 10
            print(f"   Clip {idx} ({skill}): {reason}")
        sys.exit(1)
    else:
        print("✅ Library is perfectly valid!")
        print("📊 Skill Distribution:")
        for k, v in sorted(skill_counts.items()):
            print(f"   {k}: {v} clips")

if __name__ == "__main__":
    validate("data/unified_motion_library.json")