#!/usr/bin/env python3
"""
RevEx Master Build Pipeline  v4.0
═══════════════════════════════════════════════════════════════════════════════
Combines the RevEx robot body with Shadow Hand E-series into a fully resolved,
standalone URDF that can be loaded by Blender or any physics simulator.

Features:
 - Natively resolves Xacro includes.
 - Auto-forges Shadow Hand DAE files into scaled STL files.
 - Preserves forearms for physical wrist-mounting.
 - Forces absolute Windows paths for guaranteed Blender imports.
═══════════════════════════════════════════════════════════════════════════════
"""

import sys, os, re, shutil, subprocess, logging
import xml.etree.ElementTree as ET
from types   import ModuleType
from pathlib import Path

# ──────────────────────────────────────────────────────────────────────────────
# LOGGING & PATHS
# ──────────────────────────────────────────────────────────────────────────────
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger(__name__)

MECHANICAL_DIR = Path(__file__).resolve().parent
MESH_DIR       = MECHANICAL_DIR / "meshes"
SR_ROOT        = MECHANICAL_DIR / "sr_description"
SHADOW_COMP    = SR_ROOT / "meshes" / "components"
HAND_XACRO     = SR_ROOT / "hand" / "xacro" / "hand.urdf.xacro"

REVEX_SRC      = MECHANICAL_DIR / "RevEx.urdf.xacro"
REVEX_TMP      = MECHANICAL_DIR / "_revex_patched.urdf.xacro"
REVEX_OUT      = MECHANICAL_DIR / "revex_master.urdf"

# ══════════════════════════════════════════════════════════════════════════════
# STEP 0  DEPENDENCIES & ROS MOCK
# ══════════════════════════════════════════════════════════════════════════════
def install_deps() -> None:
    for pkg in ("xacro", "trimesh", "numpy"):
        try:
            __import__(pkg)
        except ImportError:
            log.info("Installing %s …", pkg)
            subprocess.check_call(
                [sys.executable, "-m", "pip", "install", "-q", pkg],
                stdout=subprocess.DEVNULL,
            )
    log.info("Dependencies OK")

def mock_ros2() -> None:
    ament    = ModuleType("ament_index_python")
    pkgs_mod = ModuleType("ament_index_python.packages")

    def get_package_share_directory(name: str) -> str:
        if name == "sr_description":
            return str(SR_ROOT)
        raise FileNotFoundError(f"ROS package '{name}' not mocked.")

    pkgs_mod.get_package_share_directory = get_package_share_directory
    ament.packages = pkgs_mod
    sys.modules.update({"ament_index_python": ament, "ament_index_python.packages": pkgs_mod})

# ══════════════════════════════════════════════════════════════════════════════
# STEP 1  FLATTEN SHADOW HAND MESHES
# ══════════════════════════════════════════════════════════════════════════════
def flatten_shadow_meshes() -> int:
    if not SHADOW_COMP.is_dir():
        return 0

    copied, skipped = 0, 0
    for src in sorted(SHADOW_COMP.rglob("*.dae")):
        dst = MESH_DIR / src.name
        if dst.exists():
            skipped += 1
        else:
            shutil.copy2(src, dst)
            copied += 1

    log.info("Shadow meshes: %d copied, %d already present", copied, skipped)
    return copied + skipped

# ══════════════════════════════════════════════════════════════════════════════
# STEP 2  PATCH XACRO SOURCE → TEMP FILE
# ══════════════════════════════════════════════════════════════════════════════
def patch_xacro() -> Path:
    if not REVEX_SRC.exists():
        raise FileNotFoundError(f"Source xacro not found: {REVEX_SRC}")

    src = REVEX_SRC.read_text(encoding="utf-8")
    abs_hand_xacro = HAND_XACRO.as_posix()
    src = re.sub(
        r'<xacro:include\s+filename="[^"]*hand\.urdf\.xacro"\s*/>',
        f'<xacro:include filename="{abs_hand_xacro}" />',
        src,
    )
    src = src.replace("$(dirname)/", "")
    REVEX_TMP.write_text(src, encoding="utf-8")
    return REVEX_TMP

# ══════════════════════════════════════════════════════════════════════════════
# STEP 3  EXPAND XACRO → XML STRING
# ══════════════════════════════════════════════════════════════════════════════
def expand_xacro(path: Path) -> str:
    import xacro
    log.info("Running xacro processor …")
    doc = xacro.process_file(str(path))
    return doc.toprettyxml(indent="  ")

# ══════════════════════════════════════════════════════════════════════════════
# STEP 4  NORMALISE MESH PATHS
# ══════════════════════════════════════════════════════════════════════════════
def normalise_mesh_paths(xml: str) -> str:
    xml = re.sub(r'filename="package://[^/"]*/[^"]*?([^/"]+\.(?:dae|stl|DAE|STL))"', r'filename="\1"', xml)
    xml = re.sub(r'filename="(?:meshes/)([^"]+)"', r'filename="\1"', xml)
    def strip_abs(m: re.Match) -> str:
        raw = m.group(1)
        if re.match(r"[A-Za-z]:[/\\]|/[^/]", raw):
            return f'filename="{Path(raw).name}"'
        return m.group(0)
    return re.sub(r'filename="([^"]+)"', strip_abs, xml)

# ══════════════════════════════════════════════════════════════════════════════
# STEP 5  VERIFY HAND ATTACHMENT
# ══════════════════════════════════════════════════════════════════════════════
def verify_and_attach_hands(root: ET.Element) -> None:
    links = {l.get("name") for l in root.findall(".//link")}
    for required in ["rh_forearm", "lh_forearm"]:
        if required not in links:
            raise RuntimeError(f"Missing link after xacro expansion: {required}. Hands failed to generate.")
    log.info("  ✓ Hand components verified successfully.")

# ══════════════════════════════════════════════════════════════════════════════
# STEP 6  PHYSICS & BLENDER COMPATIBILITY PATCHES (The Forgemaster)
# ══════════════════════════════════════════════════════════════════════════════
def apply_physics_and_blender_fixes(root: ET.Element) -> None:
    # 1. Prevent Blender Crash (Incinerate ROS transmissions)
    for tag in ['transmission', 'gazebo']:
        for el in root.findall(f'.//{tag}'):
            for parent in root.iter():
                if el in parent:
                    parent.remove(el)
                    break

    # 2. KINEMATIC PRUNER: Destroy broken/dangling joints
    valid_links = {link.get('name') for link in root.findall('.//link') if link.get('name')}
    joints_to_remove = []
    for joint in root.findall('.//joint'):
        parent_el = joint.find('parent')
        child_el = joint.find('child')
        if (parent_el is None or child_el is None or 'type' not in joint.attrib or
            parent_el.get('link') not in valid_links or child_el.get('link') not in valid_links):
            joints_to_remove.append(joint)

    for joint in joints_to_remove:
        for parent in root.iter():
            if joint in parent:
                parent.remove(joint)
                log.info(f"  → Pruned dangling joint: '{joint.get('name')}'")
                break

    # 3. Fix zero-inertia physics engine crashes
    for inertia in root.findall('.//inertia'):
        for attr in ['ixx', 'iyy', 'izz']:
            try:
                if float(inertia.get(attr, "0.0")) <= 0.0:
                    inertia.set(attr, "0.00001")
            except ValueError:
                pass

    # 4. BLENDER BLINDNESS FIX: Auto-Forge STLs, Force Absolute Paths, AND Preserve Mirrors
    import trimesh
    import numpy as np
    
    scale_mat = trimesh.transformations.scale_matrix(0.001)

    for mesh in root.findall('.//mesh'):
        fname = mesh.get('filename', '')
        if not fname: continue
            
        basename = os.path.basename(fname)
        
        # Capture the original scale before we do anything to see if it's a Mirrored Left Hand
        orig_scale = mesh.get('scale', '1 1 1')
        
        # Auto-forge Shadow Hand DAEs into scaled STLs right now
        if basename.lower().endswith('.dae'):
            dae_path = os.path.join(MESH_DIR, basename)
            stl_basename = basename[:-4] + '.stl'
            stl_path = os.path.join(MESH_DIR, stl_basename)
            
            if not os.path.exists(stl_path) and os.path.exists(dae_path):
                log.info(f"  → Forging {basename} into STL...")
                try:
                    mesh_obj = trimesh.load(dae_path, force='mesh')
                    if isinstance(mesh_obj, trimesh.Scene):
                        geoms = list(mesh_obj.geometry.values())
                        if geoms: mesh_obj = trimesh.util.concatenate(geoms)
                    mesh_obj.apply_transform(scale_mat)
                    mesh_obj.export(stl_path)
                except Exception as e:
                    log.error(f"Failed to forge {basename}: {e}")
            
            basename = stl_basename
            
            # THE MIRROR FIX: Keep the negative sign, but drop the 0.001 size
            try:
                sx, sy, sz = map(float, orig_scale.split())
                new_sx = "-1" if sx < 0 else "1"
                new_sy = "-1" if sy < 0 else "1"
                new_sz = "-1" if sz < 0 else "1"
                
                if new_sx == "-1" or new_sy == "-1" or new_sz == "-1":
                    mesh.set('scale', f"{new_sx} {new_sy} {new_sz}")
                else:
                    if 'scale' in mesh.attrib:
                        del mesh.attrib['scale']
            except ValueError:
                if 'scale' in mesh.attrib:
                    del mesh.attrib['scale']

        # Inject pure absolute path (NO file:/// protocol)
        abs_path = os.path.abspath(os.path.join(MESH_DIR, basename)).replace('\\', '/')
        
        # Force lowercase .stl to bypass Blender's uppercase bug
        if abs_path.endswith('.STL'):
            abs_path = abs_path[:-4] + '.stl'
            
        mesh.set('filename', abs_path)
            
    log.info("  ✓ Applied physics scaling, kinematics, and Blender patches")

# ══════════════════════════════════════════════════════════════════════════════
# MAIN EXECUTION
# ══════════════════════════════════════════════════════════════════════════════
def main() -> None:
    bar = "═" * 62
    log.info(bar)
    log.info("  RevEx Master Build Pipeline  v4.0")
    log.info(bar)

    install_deps()
    mock_ros2()

    log.info("\n[1/7] Flattening Shadow Hand DAE meshes …")
    flatten_shadow_meshes()

    log.info("\n[2/7] Patching xacro include paths …")
    tmp_xacro = patch_xacro()

    log.info("\n[3/7] Expanding xacro …")
    try:
        xml_str = expand_xacro(tmp_xacro)
    finally:
        if tmp_xacro.exists():
            tmp_xacro.unlink()

    log.info("\n[4/7] Parsing and verifying XML …")
    xml_str = normalise_mesh_paths(xml_str)
    xml_body = re.sub(r"<\?xml[^>]+\?>\s*", "", xml_str, count=1)
    root = ET.fromstring(xml_body)
    
    verify_and_attach_hands(root)

    log.info("\n[5/7] Applying Forgemaster Patches …")
    seen_mats = set()
    to_remove = [mat for mat in root.findall("material") if mat.get("name") in seen_mats or seen_mats.add(mat.get("name"))]
    for mat in to_remove: root.remove(mat)
    
    apply_physics_and_blender_fixes(root)      

    log.info("\n[6/7] Validating output …")
    links = {l.get("name") for l in root.findall(".//link")}
    joints = root.findall(".//joint")
    log.info("Links: %d  |  Joints: %d", len(links), len(joints))

    log.info("\n[7/7] Writing URDF …")
    try: ET.indent(root, space="  ")
    except AttributeError: pass
    ET.ElementTree(root).write(str(REVEX_OUT), encoding="unicode", xml_declaration=True)

    log.info(bar)
    log.info("  ✓  BUILD SUCCEEDED")
    log.info("  Output → %s", REVEX_OUT)
    log.info(bar)

if __name__ == "__main__":
    main()