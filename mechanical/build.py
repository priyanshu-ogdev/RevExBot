import os
import subprocess
import sys

def compile_robot():
    print("==================================================")
    print(" RevEx Hardware Builder: Compiling Master Assembly")
    print("==================================================")
    
    # Define your files
    input_xacro = "master_assembly.xacro"
    output_urdf = "revexbot.urdf"

    # Verify the master file exists before attempting to build
    if not os.path.exists(input_xacro):
        print(f"[ERROR] Could not find '{input_xacro}' in the current directory.")
        sys.exit(1)

    # Construct the cross-platform xacro command
    # This command reads the xacro, processes the math/macros, and outputs pure XML
    command = ["xacro", input_xacro, "-o", output_urdf]

    try:
        print(f"-> Processing {input_xacro}...")
        
        # Run the command
        result = subprocess.run(command, check=True, capture_output=True, text=True)
        
        print(f"-> Success! Robot compiled perfectly.")
        print(f"-> Output saved to: {os.path.abspath(output_urdf)}")
        print("==================================================")
        
    except FileNotFoundError:
        print("[ERROR] 'xacro' is not installed or not in your system PATH.")
        print("Run: pip install xacro (or install via ROS)")
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] The xacro compiler found a syntax error in your XML.")
        print("Details:")
        print(e.stderr)
        sys.exit(1)

if __name__ == "__main__":
    compile_robot()