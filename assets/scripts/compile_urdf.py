import os
import subprocess
import sys

def compile_robot():
    print("==================================================")
    print(" RevEx Hardware Builder: Compiling Master Assembly")
    print("==================================================")

    # Determine paths relative to this script's location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_xacro = os.path.join(script_dir, "../xacro/master_assembly.xacro")
    output_dir = os.path.join(script_dir, "../urdf")
    output_urdf = os.path.join(output_dir, "revexbot1.urdf")

    # Normalize paths (resolve ..)
    input_xacro = os.path.normpath(input_xacro)
    output_urdf = os.path.normpath(output_urdf)
    output_dir = os.path.normpath(output_dir)

    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Check input file exists
    if not os.path.exists(input_xacro):
        print(f"[ERROR] Could not find '{input_xacro}'.")
        sys.exit(1)

    # Command
    command = ["xacro", input_xacro, "-o", output_urdf]

    try:
        print(f"-> Processing {input_xacro}...")
        result = subprocess.run(command, check=True, capture_output=True, text=True)

        print(f"-> Success! Robot compiled perfectly.")
        print(f"-> Output saved to: {output_urdf}")
        print("==================================================")

    except FileNotFoundError:
        print("[ERROR] 'xacro' is not installed or not in your system PATH.")
        print("Run: pip install xacro (or install via ROS)")
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] xacro compilation failed with exit code {e.returncode}")
        print("STDERR:")
        print(e.stderr)
        sys.exit(1)

if __name__ == "__main__":
    compile_robot()