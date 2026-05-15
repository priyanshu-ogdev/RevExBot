import sys
import os
import xml.etree.ElementTree as ET
from types import ModuleType
import logging

# ==========================================
# 0. SETUP & LOGGING
# ==========================================
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

# Ensure xacro is installed
try:
    import xacro
except ImportError:
    logger.info("Installing xacro parser...")
    import subprocess
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', '-q', 'xacro'])
    import xacro

# ==========================================
# 1. DYNAMIC PATH CONFIGURATIONS
# ==========================================
# Grabs the exact absolute path of the mechanical/ folder where this script lives
MECHANICAL_DIR = os.path.dirname(os.path.abspath(__file__))
SR_DESCRIPTION_PATH = os.path.join(MECHANICAL_DIR, "sr_description")
XACRO_DIR = os.path.join(SR_DESCRIPTION_PATH, "hand", "xacro")
MESH_DIR = os.path.join(MECHANICAL_DIR, "meshes")

# Input/Output definitions
ASIMOV_URDF = os.path.join(MESH_DIR, "RevEx.urdf")
REVEX_OUTPUT = os.path.join(MECHANICAL_DIR, "revex_master.urdf")  # Placed in root mechanical folder

# ==========================================
# 2. THE ROS 2 BYPASS
# ==========================================
# Tricks the Xacro compiler into using our local Windows folder instead of Linux ROS
mock_ament = ModuleType("ament_index_python")
mock_packages = ModuleType("packages")

def get_package_share_directory(package_name):
    if package_name == "sr_description":
        return SR_DESCRIPTION_PATH
    raise ValueError(f"Unknown package: {package_name}")

mock_packages.get_package_share_directory = get_package_share_directory
mock_ament.packages = mock_packages
sys.modules["ament_index_python"] = mock_ament
sys.modules["ament_index_python.packages"] = mock_packages

# ==========================================
# 3. HAND COMPILATION & SCALING
# ==========================================
def build_hand_tree(side):
    prefix = "rh" if side == "right" else "lh"
    main_xacro = os.path.join(XACRO_DIR, "hand.urdf.xacro")
    phantom_path = os.path.join(XACRO_DIR, f"_phantom_{side}.urdf.xacro")
    
    # We spoon-feed the macro parameters to prevent the "No Fingers" bug
    phantom_content = f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="shadow_{side}">
    <xacro:include filename="{main_xacro}" />
    <xacro:sr_hand 
        side="{side}" 
        hand_type="hand_e" 
        hand_version="E3M5" 
        fingers="th,ff,mf,rf,lf"
        tip_sensors="pst"
        mid_sensors="none"
        prox_sensors="none"
        palm_sensor="none"
        mounting_plate="false" />
</robot>
"""
    with open(phantom_path, 'w') as f:
        f.write(phantom_content)
        
    try:
        # Compile Xacro to standard XML
        doc = xacro.process_file(phantom_path)
        root = ET.fromstring(doc.toprettyxml())
        
        # Sanitize and scale the hand meshes
        for mesh in root.findall('.//mesh'):
            fname = mesh.get('filename', '')
            if 'package://sr_description/' in fname:
                # Changes path to: sr_description/meshes/components/...
                # This works perfectly because revex_master.urdf is sitting right next to the sr_description folder
                new_fname = fname.replace('package://', '')
                mesh.set('filename', new_fname)
                
                # INJECT FIX: Scale the DAE by 1000x to fix the Millimeter Bug without losing color data
                if new_fname.lower().endswith('.dae'):
                    mesh.set('scale', '0.001 0.001 0.001')
                    
        # Fix zero-inertia engine crash bug
        for inertia in root.findall('.//inertia'):
            for attr in ['ixx', 'iyy', 'izz']:
                try:
                    if float(inertia.get(attr, "0.0")) <= 0.0:
                        inertia.set(attr, "0.00001")
                except ValueError:
                    pass
                    
        return root
    except Exception as e:
        logger.error(f"Failed to build {side} hand: {e}")
        sys.exit(1)
    finally:
        if os.path.exists(phantom_path):
            os.remove(phantom_path)

# ==========================================
# 4. MASTER ASSEMBLY
# ==========================================
def assemble_revex():
    logger.info("Loading base RevEx chassis...")
    try:
        revex_tree = ET.parse(ASIMOV_URDF)
        revex_root = revex_tree.getroot()
    except Exception as e:
        logger.error(f"ERROR parsing {ASIMOV_URDF}: {e}")
        sys.exit(1)
        
    # FIX ASIMOV PATHS:
    # Since revex_master.urdf will be in mechanical/, it needs to look down into meshes/ for the chassis parts.
    for mesh in revex_root.findall('.//mesh'):
        fname = mesh.get('filename', '')
        if fname and not fname.startswith('meshes/') and not fname.startswith('sr_description/'):
            mesh.set('filename', f"meshes/{fname}")
            
    logger.info("Compiling Shadow Hands directly from DAEs (Preserving Colors!)...")
    hands = {'right': build_hand_tree('right'), 'left': build_hand_tree('left')}
    
    logger.info("Integrating hands and preventing material duplication...")
    existing_materials = {m.get('name') for m in revex_root.findall('.//material')}
    
    for side in ['right', 'left']:
        hand_root = hands[side]
        for link in hand_root.findall('.//link'): revex_root.append(link)
        for joint in hand_root.findall('.//joint'): revex_root.append(joint)
        for material in hand_root.findall('.//material'):
            mat_name = material.get('name')
            if mat_name not in existing_materials:
                revex_root.append(material)
                existing_materials.add(mat_name)
                
    logger.info("Forging Digital Welds (Wrist -> Palm)...")
    prefix_map = {'right': 'rh', 'left': 'lh'}
    wrist_links = {'right': 'right_wrist_yaw_link', 'left': 'left_wrist_yaw_link'}
    
    for side, wrist_link in wrist_links.items():
        # Validate that Asimov's wrist exists before bolting
        if revex_root.find(f'.//link[@name="{wrist_link}"]') is None:
            logger.error(f"CRITICAL: {wrist_link} missing from base URDF!")
            sys.exit(1)
            
        joint = ET.Element("joint", name=f"{side}_wrist_to_hand", type="fixed")
        ET.SubElement(joint, "parent", link=wrist_link)
        ET.SubElement(joint, "child", link=f"{prefix_map[side]}_palm")
        ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
        revex_root.append(joint)
        
    logger.info("Exporting unified master URDF...")
    revex_tree._setroot(revex_root)
    revex_tree.write(REVEX_OUTPUT, encoding='utf-8', xml_declaration=True)
    
    logger.info("=" * 60)
    logger.info(f"✓ SUCCESS! File saved to: {REVEX_OUTPUT}")
    logger.info("You can now open Isaac Sim and import 'revex_master.urdf'.")
    logger.info("=" * 60)

if __name__ == "__main__":
    assemble_revex()