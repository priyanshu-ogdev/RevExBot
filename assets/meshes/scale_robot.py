import xml.etree.ElementTree as ET

def scale_urdf(input_file, output_file, scale_factor):
    # Parse the mathematical tree
    tree = ET.parse(input_file)
    root = tree.getroot()

    # 1. Scale the kinematic distances between joints
    for origin in root.findall('.//origin'):
        xyz = origin.get('xyz')
        if xyz:
            # Extract, multiply, and format back to 6 decimal places for physics precision
            coords = [float(x) * scale_factor for x in xyz.split()]
            origin.set('xyz', f"{coords[0]:.6f} {coords[1]:.6f} {coords[2]:.6f}")

    # 2. Scale the visual bounding meshes
    for mesh in root.findall('.//mesh'):
        mesh.set('scale', f"{scale_factor} {scale_factor} {scale_factor}")

    # Export the new working product
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"Success! {output_file} generated with a native scale of {scale_factor}x.")

if __name__ == "__main__":
    # 5 feet / 3.93 feet (1.2 meters) = 1.272
    SCALE_MULTIPLIER = 1.272 
    scale_urdf('asimov.urdf', 'asimov_5ft.urdf', SCALE_MULTIPLIER)