import os
import mujoco

def sync_urdf_to_mjcf():
    print("========================================================")
    print("🏭 REVEXBOT FORGE: URDF -> MJCF COMPILER")
    print("========================================================")

    # Paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.abspath(os.path.join(script_dir, "..", "urdf", "revexbot1.urdf"))
    mjcf_out_path = os.path.abspath(os.path.join(script_dir, "..", "mjcf", "revexbot_synced.xml"))

    if not os.path.exists(urdf_path):
        print(f"❌ FATAL: Source URDF not found at {urdf_path}")
        return

    try:
        # MuJoCo natively parses the URDF, resolves masses, and converts <mimic> tags to <equality> tags
        model = mujoco.MjModel.from_xml_path(urdf_path)
        
        # Save the mathematically perfect MJCF
        mujoco.mj_saveLastXML(mjcf_out_path, model)
        
        print(f"✅ SUCCESS: Synced MJCF generated at {mjcf_out_path}")
    except Exception as e:
        print(f"❌ COMPILATION FAILED: {e}")

if __name__ == "__main__":
    sync_urdf_to_mjcf()