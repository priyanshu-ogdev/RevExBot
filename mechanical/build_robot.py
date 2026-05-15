import sys
import os
import re
import xml.etree.ElementTree as ET
from types import ModuleType
import logging
from collections import defaultdict

# ==========================================
# 0. DEPENDENCY INSTALLER
# ==========================================
def ensure_packages():
    """Install missing packages automatically"""
    required = ['xacro', 'trimesh', 'numpy']
    missing = []
    
    for pkg in required:
        try:
            __import__(pkg)
        except ImportError:
            missing.append(pkg)
    
    if missing:
        logger.info(f"Installing missing packages: {', '.join(missing)}")
        import subprocess
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', '-q'] + missing)
        logger.info("✓ Packages installed")

# Initialize logging FIRST
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(message)s'
)
logger = logging.getLogger(__name__)

# Install dependencies
ensure_packages()

# NOW import the packages
import xacro
import trimesh
import numpy as np

# ==========================================
# 1. THE ROS 2 BYPASS
# ==========================================
mock_ament = ModuleType("ament_index_python")
mock_packages = ModuleType("packages")
def get_package_share_directory(package_name):
    if package_name == "sr_description":
        return "C:/Projects/RevExBot/RevEx_Phase1_Hardware/sr_description"
    raise ValueError(f"Unknown package: {package_name}")
mock_packages.get_package_share_directory = get_package_share_directory
mock_ament.packages = mock_packages
sys.modules["ament_index_python"] = mock_ament
sys.modules["ament_index_python.packages"] = mock_packages

# ==========================================
# 2. PATH CONFIGURATIONS
# ==========================================
XACRO_DIR = "C:/Projects/RevExBot/RevEx_Phase1_Hardware/sr_description/hand/xacro"
MESH_DIR = r"C:\Projects\RevExBot\RevEx_Phase1_Hardware\mechanical\meshes"
SR_DESCRIPTION_PATH = "C:/Projects/RevExBot/RevEx_Phase1_Hardware/sr_description"
ASIMOV_URDF = os.path.join(MESH_DIR, "RevEx.urdf")
REVEX_OUTPUT = os.path.join(MESH_DIR, "revex_master.urdf")

# Mesh resolution cache
MESH_CACHE = {}

# ==========================================
# 3. MESH PATH RESOLUTION
# ==========================================
def resolve_mesh_path(mesh_ref):
    """Resolve package:// URIs and relative paths to absolute paths in MESH_DIR"""
    if not mesh_ref:
        return None
    
    if mesh_ref in MESH_CACHE:
        return MESH_CACHE[mesh_ref]
    
    # Handle package:// URIs
    if mesh_ref.startswith("package://sr_description/"):
        # Extract relative path from package URI
        rel_path = mesh_ref.replace("package://sr_description/", "")
        # Get mesh filename
        filename = os.path.basename(rel_path)
        full_path = os.path.join(MESH_DIR, filename)
        MESH_CACHE[mesh_ref] = full_path
        return full_path
    
    elif mesh_ref.startswith("package://"):
        filename = os.path.basename(mesh_ref)
        full_path = os.path.join(MESH_DIR, filename)
        MESH_CACHE[mesh_ref] = full_path
        return full_path
    
    # Handle absolute paths (already resolved)
    elif os.path.isabs(mesh_ref):
        if os.path.exists(mesh_ref):
            MESH_CACHE[mesh_ref] = mesh_ref
            return mesh_ref
    
    # Handle relative paths
    full_path = os.path.join(MESH_DIR, os.path.basename(mesh_ref))
    if os.path.exists(full_path):
        MESH_CACHE[mesh_ref] = full_path
        return full_path
    
    logger.warning(f"Mesh not found: {mesh_ref}")
    return None

def get_mesh_bounds(mesh_path):
    """Get mesh bounds (min/max) for validation"""
    if not os.path.exists(mesh_path):
        return None
    try:
        mesh = trimesh.load(mesh_path, force='mesh')
        if isinstance(mesh, trimesh.Scene):
            meshes = [geom for geom in mesh.geometry.values()]
            if not meshes: return None
            mesh = trimesh.util.concatenate(meshes)
        bounds = mesh.bounds
        return {
            'min': bounds[0].tolist(),
            'max': bounds[1].tolist(),
            'size': (bounds[1] - bounds[0]).tolist()
        }
    except Exception as e:
        logger.warning(f"Could not load mesh {mesh_path}: {e}")
        return None

# ==========================================
# 4. AUTO-DISCOVERY XACRO COMPILER (NATIVE SCALE)
# ==========================================
def auto_build_hand(side):
    """Build hand from xacro with robust macro discovery and parameter handling"""
    logger.info(f"Initiating Auto-Discovery for {side} hand...")
    prefix = "rh" if side == "right" else "lh"
    reflect = "1.0" if side == "right" else "-1.0"
    
    # Discover all available xacro macros
    candidates = []
    for fname in os.listdir(XACRO_DIR):
        if not fname.endswith(".xacro"):
            continue
        try:
            with open(os.path.join(XACRO_DIR, fname), 'r') as f:
                content = f.read()
                # Find macro definitions with their parameters
                matches = re.findall(
                    r'<xacro:macro\s+name="([^"]+)"(?:\s+params="([^"]*)")?',
                    content
                )
                for m_name, m_params in matches:
                    candidates.append((m_name, m_params, fname))
        except Exception as e:
            logger.warning(f"Error parsing {fname}: {e}")
            continue
    
    if not candidates:
        logger.error("No xacro macros found!")
        sys.exit(1)
    
    logger.info(f"Discovered {len(candidates)} macro(s)")
    
    # Prioritize hand macros (sr_hand, hand_e, shadowhand_e, etc.)
    def sort_key(c):
        name = c[0].lower()
        # Prioritize full hand macros over component macros
        if any(x in name for x in ['sr_hand', 'hand_e', 'hand_c', 'shadowhand']):
            return (0, -len(name))  # Shorter names first among hand macros
        elif 'hand' in name:
            return (1, 0)
        else:
            return (2, 0)
    
    candidates.sort(key=sort_key)
    
    last_error = None
    for m_name, m_params, fname in candidates:
        logger.debug(f"Attempting macro: {m_name} from {fname}")
        
        # Build macro parameters from function signature
        call_attrs = build_macro_params(m_params, prefix, side, reflect)
        macro_call = f"<xacro:{m_name} {' '.join(call_attrs)} />"
        
        # Create temporary xacro file
        phantom_path = os.path.join(XACRO_DIR, f"_phantom_{side}.urdf.xacro")
        phantom_content = f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="shadow_{side}">
    <xacro:property name="side" value="{side}" />
    <xacro:property name="prefix" value="{prefix}_" />
    <xacro:property name="reflect" value="{reflect}" />
    <xacro:property name="hand_type" value="hand_e" />
    <xacro:property name="hand_version" value="E3M5" />
    <xacro:property name="fingers" value="th,ff,mf,rf,lf" />
    <xacro:property name="tip_sensors" value="pst" />
    <xacro:property name="mid_sensors" value="none" />
    <xacro:property name="prox_sensors" value="none" />
    <xacro:property name="palm_sensor" value="none" />
    <xacro:property name="mounting_plate" value="false" />
    <xacro:property name="visual" value="true" />
    <xacro:include filename="{fname}" />
    {macro_call}
</robot>
"""
        
        with open(phantom_path, 'w') as f:
            f.write(phantom_content)
        
        try:
            # Process xacro
            doc = xacro.process_file(phantom_path)
            root = ET.fromstring(doc.toprettyxml())
            links = root.findall('.//link')
            joints = root.findall('.//joint')
            
            logger.info(f"   ✓ Macro {m_name} generated {len(links)} links, {len(joints)} joints")
            
            # Validate significant geometry
            if len(links) > 10:
                logger.info(f"   ✓ TARGET ACQUIRED: Executed <xacro:{m_name}> ({fname})")
                
                # Update mesh references
                update_mesh_paths(root)
                return root
            else:
                logger.debug(f"   ✗ Insufficient geometry: {len(links)} links")
                
        except Exception as e:
            logger.debug(f"   ✗ Failed: {str(e)[:100]}")
            last_error = e
        finally:
            if os.path.exists(phantom_path):
                os.remove(phantom_path)
    
    logger.error(f"FATAL: No macro produced sufficient geometry")
    if last_error:
        logger.error(f"Last error: {last_error}")
    sys.exit(1)

def build_macro_params(param_string, prefix, side, reflect):
    """Build macro call parameters from xacro macro signature"""
    call_attrs = []
    
    if not param_string:
        return call_attrs
    
    # Parse parameter names and defaults
    params = {}
    for p in param_string.split():
        # Handle params with defaults: "name:=default" or "name:=^"
        p_match = re.match(r'([a-zA-Z_][a-zA-Z0-9_]*)', p)
        if p_match:
            params[p_match.group(1)] = True
    
    # Map parameters to values
    param_map = {
        'prefix': prefix + '_',
        'reflect': reflect,
        'side': side,
        'hand_type': 'hand_e',
        'hand_version': 'E3M5',
        'fingers': 'th,ff,mf,rf,lf',
        'tip_sensors': 'pst',
        'mid_sensors': 'none',
        'prox_sensors': 'none',
        'palm_sensor': 'none',
        'mounting_plate': 'false',
        'visual': 'true',
        'is_lite': '0',
        'parent': f'{prefix}_palm'
    }
    
    for p_name in params:
        if p_name in param_map:
            value = param_map[p_name]
            call_attrs.append(f'{p_name}="{value}"')
        else:
            call_attrs.append(f'{p_name}=""')
    
    return call_attrs

def update_mesh_paths(root):
    """Update mesh references from package:// to local files"""
    mesh_updates = 0
    for mesh in root.findall('.//mesh'):
        filename_attr = mesh.get('filename')
        if not filename_attr:
            continue
        
        resolved = resolve_mesh_path(filename_attr)
        if resolved:
            # Use just the basename for mesh references
            mesh.set('filename', os.path.basename(resolved))
            mesh_updates += 1
        else:
            # Try using just the basename
            basename = os.path.basename(filename_attr)
            if os.path.exists(os.path.join(MESH_DIR, basename)):
                mesh.set('filename', basename)
                mesh_updates += 1
    
    logger.debug(f"   → Updated {mesh_updates} mesh references")

# ==========================================
# 5. MESH CONVERSION (OPTIMIZED FORGEMASTER)
# ==========================================
def analyze_dae_mesh(dae_path):
    """Analyze DAE mesh to determine appropriate transforms"""
    try:
        mesh = trimesh.load(dae_path, force='mesh')
        if isinstance(mesh, trimesh.Scene):
            geoms = list(mesh.geometry.values())
            if not geoms:
                return None
            mesh = trimesh.util.concatenate(geoms)
        
        bounds = mesh.bounds
        size = bounds[1] - bounds[0]
        
        return {
            'bounds_min': bounds[0],
            'bounds_max': bounds[1],
            'size': size,
            'center': (bounds[0] + bounds[1]) / 2
        }
    except Exception as e:
        logger.warning(f"Failed to analyze {os.path.basename(dae_path)}: {e}")
        return None

def reforge_meshes():
    """Convert DAE meshes to STL with validated transforms"""
    logger.info("Forging DAE into STL with physics-aware transforms...")
    
    # Standard transformation matrices
    # Note: These assume DAE meshes are in millimeters (10-meter finger bug)
    # Scale factor: 0.001 (convert mm to m)
    scale_mat = trimesh.transformations.scale_matrix(0.001)
    
    # Rotation: Y_UP to Z_UP (common DAE to URDF convention)
    # Rotate 90° around X-axis to align Y→Z
    rot_mat = trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0])
    
    # Combined transform
    master_transform = np.dot(rot_mat, scale_mat)
    
    converted_count = 0
    skipped_count = 0
    failed_meshes = []
    
    for filename in sorted(os.listdir(MESH_DIR)):
        if not filename.endswith(".dae"):
            continue
        
        dae_path = os.path.join(MESH_DIR, filename)
        stl_filename = filename.replace(".dae", ".stl")
        stl_path = os.path.join(MESH_DIR, stl_filename)
        
        # Skip if STL already exists
        if os.path.exists(stl_path):
            logger.debug(f"  ✓ {stl_filename} (already exists)")
            skipped_count += 1
            continue
        
        try:
            logger.debug(f"  → Converting {filename}...")
            
            # Analyze source mesh
            source_info = analyze_dae_mesh(dae_path)
            if not source_info:
                logger.warning(f"  ✗ Could not analyze {filename}")
                failed_meshes.append(filename)
                continue
            
            # Load and process mesh
            mesh = trimesh.load(dae_path, force='mesh')
            if isinstance(mesh, trimesh.Scene):
                geoms = list(mesh.geometry.values())
                if not geoms:
                    logger.warning(f"  ✗ {filename} has no geometry")
                    failed_meshes.append(filename)
                    continue
                mesh = trimesh.util.concatenate(geoms)
            
            # Apply transform
            mesh.apply_transform(master_transform)
            
            # Validate result
            result_info = analyze_dae_mesh(dae_path)  # Load before transform for comparison
            result_bounds = mesh.bounds
            result_size = result_bounds[1] - result_bounds[0]
            
            # Check for degenerate meshes (must have at least some dimension)
            # Allow extremely small parts (connectors, adapters) - threshold: 0.00001 m (10 micrometers)
            min_dimension = np.min(result_size)
            if min_dimension < 0.000001:  # Less than 1 micrometer is truly degenerate
                logger.warning(f"  ✗ {filename} produced degenerate mesh (min dim: {min_dimension:.2e}m)")
                failed_meshes.append(filename)
                continue
            
            # Export STL with proper normals
            mesh.export(stl_path)
            
            # Verify export
            if os.path.exists(stl_path) and os.path.getsize(stl_path) > 100:
                converted_count += 1
                # Log size category
                size_category = ""
                if np.all(result_size > 0.01):
                    size_category = "(standard)"
                elif np.all(result_size > 0.001):
                    size_category = "(small)"
                elif np.all(result_size > 0.0001):
                    size_category = "(tiny - connector/adapter)"
                else:
                    size_category = "(micro)"
                logger.debug(f"  \u2713 {stl_filename} {size_category}")
            else:
                logger.warning(f"  ✗ Export failed for {filename}")
                failed_meshes.append(filename)
                if os.path.exists(stl_path):
                    os.remove(stl_path)
                    
        except Exception as e:
            logger.warning(f"  ✗ Failed to forge {filename}: {str(e)[:100]}")
            failed_meshes.append(filename)
    
    logger.info(f"  → Forged {converted_count} STL meshes, skipped {skipped_count} existing")
    if failed_meshes:
        logger.warning(f"  → Failed meshes: {', '.join(failed_meshes[:5])}" + 
                      ("..." if len(failed_meshes) > 5 else ""))
    
    return converted_count > 0 or skipped_count > 0

# ==========================================
# 6. MASTER ASSEMBLY & VALIDATION
# ==========================================
def find_wrist_attachment_points(asimov_root):
    """Find wrist links and validate they exist"""
    wrist_links = {
        'right': 'right_wrist_yaw_link',
        'left': 'left_wrist_yaw_link'
    }
    
    found_links = {}
    for side, link_name in wrist_links.items():
        link = asimov_root.find(f'.//link[@name="{link_name}"]')
        if link is not None:
            found_links[side] = link_name
            logger.debug(f"  ✓ Found {side} wrist: {link_name}")
        else:
            logger.warning(f"  ✗ Missing {side} wrist link: {link_name}")
    
    return found_links

def remove_element_safely(root, element):
    """Safely remove an element from tree by finding its parent"""
    for parent in root.iter():
        if element in parent:
            parent.remove(element)
            return True
    return False

def sanitize_hand_tree(hand_root, side):
    """Clean up hand tree before integration"""
    prefix = "rh" if side == "right" else "lh"
    
    # Remove invalid joints (missing type attribute)
    invalid_joints = []
    for joint in hand_root.findall('.//joint'):
        if 'type' not in joint.attrib:
            invalid_joints.append(joint)
    
    for joint in invalid_joints:
        if remove_element_safely(hand_root, joint):
            if 'name' in joint.attrib:
                logger.debug(f"  → Removed invalid joint: {joint.attrib['name']}")
    
    # Fix zero-inertia values (physics engines don't like 0 inertia)
    for inertia in hand_root.findall('.//inertia'):
        for attr in ['ixx', 'iyy', 'izz']:
            try:
                val = float(inertia.get(attr, "0.0"))
                if val <= 0.0:
                    # Set to small but non-zero value
                    inertia.set(attr, "0.000010")
            except ValueError:
                pass
    
    # Update mesh filename references to use STL instead of DAE
    mesh_updates = 0
    for mesh in hand_root.findall('.//mesh'):
        filename = mesh.get('filename', '')
        if filename.endswith('.dae'):
            new_filename = filename.replace('.dae', '.stl')
            mesh.set('filename', new_filename)
            mesh_updates += 1
        # Remove package URIs, use bare filename
        elif 'package://' in filename:
            mesh.set('filename', os.path.basename(filename))
    
    logger.info(f"  → Sanitized {side} hand: fixed {len(invalid_joints)} joints, "
               f"updated {mesh_updates} mesh refs")

def assemble_revex():
    """Master assembly: combine Asimov chassis with hands"""
    logger.info("=" * 60)
    logger.info("REVEX MASTER ASSEMBLY")
    logger.info("=" * 60)
    
    # Load base chassis
    logger.info("[1/5] Loading base Asimov chassis...")
    try:
        asimov_tree = ET.parse(ASIMOV_URDF)
        asimov_root = asimov_tree.getroot()
        asimov_links = len(asimov_root.findall('.//link'))
        asimov_joints = len(asimov_root.findall('.//joint'))
        logger.info(f"  ✓ Loaded: {asimov_links} links, {asimov_joints} joints")
    except FileNotFoundError:
        logger.error(f"ERROR: {ASIMOV_URDF} not found")
        sys.exit(1)
    except Exception as e:
        logger.error(f"ERROR parsing URDF: {e}")
        sys.exit(1)
    
    # Find wrist attachment points
    logger.info("[2/5] Validating wrist attachment points...")
    wrist_links = find_wrist_attachment_points(asimov_root)
    if len(wrist_links) < 2:
        logger.error("ERROR: Could not find both wrist links in Asimov model")
        sys.exit(1)
    
    # Build hands
    logger.info("[3/5] Building hand modules...")
    hands = {}
    for side in ['right', 'left']:
        logger.info(f"  Building {side} hand...")
        hands[side] = auto_build_hand(side)
        hand_links = len(hands[side].findall('.//link'))
        hand_joints = len(hands[side].findall('.//joint'))
        logger.info(f"    Generated: {hand_links} links, {hand_joints} joints")
    
    # Integrate hands
    logger.info("[4/5] Integrating hands into chassis...")
    
    # Track all materials to avoid duplicates
    existing_materials = {m.get('name'): m for m in asimov_root.findall('.//material')}
    added_materials = set()
    
    for side in ['right', 'left']:
        logger.info(f"  Integrating {side} hand...")
        hand_root = hands[side]
        
        # Sanitize hand tree
        sanitize_hand_tree(hand_root, side)
        
        # Add hand links
        hand_links = hand_root.findall('.//link')
        for link in hand_links:
            asimov_root.append(link)
        
        # Add hand joints
        hand_joints = hand_root.findall('.//joint')
        for joint in hand_joints:
            asimov_root.append(joint)
        
        # Add materials (avoid duplicates)
        for material in hand_root.findall('.//material'):
            mat_name = material.get('name')
            if mat_name not in existing_materials and mat_name not in added_materials:
                asimov_root.append(material)
                added_materials.add(mat_name)
    
    # Create wrist-to-hand fixed joints
    logger.info("[4b/5] Creating wrist attachment joints...")
    
    prefix_map = {'right': 'rh', 'left': 'lh'}
    attachment_joints = []
    
    for side, wrist_link in wrist_links.items():
        prefix = prefix_map[side]
        hand_base_link = f"{prefix}_palm"  # Typical first hand link
        
        joint_name = f"{side}_wrist_to_hand"
        joint = ET.Element("joint", name=joint_name, type="fixed")
        ET.SubElement(joint, "parent", link=wrist_link)
        ET.SubElement(joint, "child", link=hand_base_link)
        ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
        
        asimov_root.append(joint)
        attachment_joints.append(joint_name)
        logger.info(f"  ✓ Created: {joint_name}")
    
    # Final sanitization of complete model
    logger.info("[5/5] Final sanitization of complete model...")
    
    # Remove any duplicate materials (only top-level materials)
    mat_names_seen = set()
    duplicate_materials = []
    for material in asimov_root.findall('./material'):  # Only direct children
        mat_name = material.get('name')
        if mat_name in mat_names_seen:
            duplicate_materials.append(material)
        else:
            mat_names_seen.add(mat_name)
    
    for material in duplicate_materials:
        if material in asimov_root:
            asimov_root.remove(material)
    
    if duplicate_materials:
        logger.info(f"  → Removed {len(duplicate_materials)} duplicate material definitions")
    
    # Validate final structure
    final_links = len(asimov_root.findall('.//link'))
    final_joints = len(asimov_root.findall('.//joint'))
    final_materials = len(asimov_root.findall('.//material'))
    
    logger.info(f"  ✓ Final model: {final_links} links, {final_joints} joints, {final_materials} materials")
    
    # Verify no broken mesh references
    broken_meshes = 0
    for mesh in asimov_root.findall('.//mesh'):
        filename = mesh.get('filename', '')
        if filename and not filename.startswith('package://'):
            mesh_path = os.path.join(MESH_DIR, filename)
            if not os.path.exists(mesh_path):
                logger.warning(f"  ⚠ Missing mesh: {filename}")
                broken_meshes += 1
    
    if broken_meshes > 0:
        logger.warning(f"  ⚠ {broken_meshes} mesh files not found (may be in different location)")
    
    # Export final URDF
    logger.info(f"Exporting final model...")
    asimov_tree._setroot(asimov_root)
    asimov_tree.write(REVEX_OUTPUT, encoding='utf-8', xml_declaration=True)
    
    logger.info("=" * 60)
    logger.info(f"✓ SUCCESS! Final model generated:")
    logger.info(f"  {REVEX_OUTPUT}")
    logger.info(f"  Links: {final_links}, Joints: {final_joints}")
    logger.info("=" * 60)

if __name__ == "__main__":
    try:
        logger.info("Starting REVEX compilation pipeline...")
        logger.info(f"Workspace: {MESH_DIR}")
        
        # Phase 1: Mesh conversion
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 1: DAE→STL MESH CONVERSION")
        logger.info("=" * 60)
        if reforge_meshes():
            logger.info("✓ Mesh conversion completed")
        else:
            logger.warning("⚠ No new meshes were converted")
        
        # Phase 2: Model assembly
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 2: MODEL ASSEMBLY")
        logger.info("=" * 60)
        assemble_revex()
        
        logger.info("\n✓ REVEX compilation pipeline completed successfully!")
        
    except KeyboardInterrupt:
        logger.error("\n⚠ Pipeline interrupted by user")
        sys.exit(1)
    except Exception as e:
        logger.error(f"\n✗ FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)