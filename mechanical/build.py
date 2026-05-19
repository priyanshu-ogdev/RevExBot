import os
import xml.etree.ElementTree as ET

# Set your directories
URDF_DIR = "C:/Projects/RevExBot/RevEx_Phase1_Hardware/mechanical/urdf"
MESH_DIR = "C:/Projects/RevExBot/RevEx_Phase1_Hardware/mechanical/meshes"

# The Xacro files to parse
XACRO_FILES = [
    os.path.join(URDF_DIR, "master_assembly.xacro"),
    os.path.join(URDF_DIR, "shadow_hand.xacro")
]

def get_used_meshes(xacro_files):
    used_meshes = set()
    for file in xacro_files:
        if not os.path.exists(file):
            print(f"Warning: Could not find {file}")
            continue
            
        tree = ET.parse(file)
        root = tree.getroot()
        
        # Find all mesh tags and extract the filename
        for mesh in root.iter('mesh'):
            filename = mesh.get('filename')
            if filename:
                base_name = os.path.basename(filename).lower()
                
                # --- THE FIX: Unpack Xacro Variables ---
                if "${prefix}" in base_name:
                    used_meshes.add(base_name.replace("${prefix}", "rh_"))
                    used_meshes.add(base_name.replace("${prefix}", "lh_"))
                else:
                    used_meshes.add(base_name)
                    
    return used_meshes

def clean_mesh_directory():
    print("Parsing Xacro files for active meshes...")
    used_meshes = get_used_meshes(XACRO_FILES)
    
    if not used_meshes:
        print("Error: No meshes found in Xacro files. Aborting to prevent mass deletion.")
        return

    print(f"Found {len(used_meshes)} unique meshes active in the URDF.")
    
    # --- THE FIX: Native OS listing prevents double-counting on Windows ---
    to_delete = []
    if os.path.exists(MESH_DIR):
        for file_name in os.listdir(MESH_DIR):
            if file_name.lower().endswith(('.stl', '.dae')):
                if file_name.lower() not in used_meshes:
                    to_delete.append(os.path.join(MESH_DIR, file_name))

    if not to_delete:
        print("Mesh directory is already clean. No unused files found.")
        return

    print("\nThe following unused files will be DELETED:")
    for f in sorted(to_delete):
        print(f"  - {os.path.basename(f)}")

    confirm = input("\nType 'DELETE' to confirm: ")
    if confirm == 'DELETE':
        for f in to_delete:
            os.remove(f)
        print("Cleanup complete.")
    else:
        print("Operation cancelled.")

if __name__ == "__main__":
    clean_mesh_directory()