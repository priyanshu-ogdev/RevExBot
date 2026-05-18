#!/usr/bin/env python3
"""
Convert a Collada (.dae) file to STL (.stl), optionally applying a rotation
around a pivot point to bake a specific orientation into the mesh.
"""

import os
import argparse
import trimesh
import numpy as np
from math import radians

def convert_dae_to_stl(input_dae, output_stl, 
                       rotate_axis=None, rotate_angle=0.0, 
                       pivot=[0, 0, 0],
                       scale=1.0):
    """
    Convert DAE to STL, optionally rotate around a pivot and scale.
    
    Args:
        input_dae (str): Path to input .dae file.
        output_stl (str): Path for output .stl file.
        rotate_axis (str): 'x', 'y', or 'z' – axis of rotation.
        rotate_angle (float): Rotation angle in degrees.
        pivot (list): [x, y, z] point in mesh's local coordinates to rotate around.
        scale (float): Uniform scale factor.
    """
    if not os.path.exists(input_dae):
        raise FileNotFoundError(f"Input file not found: {input_dae}")

    # Load mesh (handles scenes)
    mesh = trimesh.load(input_dae, force='mesh')
    if isinstance(mesh, trimesh.Scene):
        # If it's a scene, concatenate all geometries into one mesh
        geometries = [g for g in mesh.geometry.values()]
        if not geometries:
            raise ValueError("No geometry found in the DAE file.")
        mesh = trimesh.util.concatenate(geometries)

    # Apply uniform scaling
    if scale != 1.0:
        mesh.apply_scale(scale)
        print(f"Applied scale factor: {scale}")

    # Apply rotation around pivot
    if rotate_axis and rotate_angle != 0.0:
        pivot = np.array(pivot, dtype=float)
        angle_rad = radians(rotate_angle)
        if rotate_axis.lower() == 'x':
            rot_mat = trimesh.transformations.rotation_matrix(angle_rad, [1, 0, 0])
        elif rotate_axis.lower() == 'y':
            rot_mat = trimesh.transformations.rotation_matrix(angle_rad, [0, 1, 0])
        elif rotate_axis.lower() == 'z':
            rot_mat = trimesh.transformations.rotation_matrix(angle_rad, [0, 0, 1])
        else:
            raise ValueError("rotate_axis must be 'x', 'y', or 'z'")

        # Translate to origin, rotate, translate back
        transform = np.eye(4)
        transform[:3, 3] = -pivot
        transform = rot_mat @ transform
        transform[:3, 3] += pivot
        mesh.apply_transform(transform)
        print(f"Rotated {rotate_angle}° around {rotate_axis.upper()}-axis, pivot={pivot}")

    # Export to STL (binary by default)
    mesh.export(output_stl)
    print(f"✅ Converted {input_dae} -> {output_stl}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert DAE to STL with optional rotation and scaling.")
    parser.add_argument("input", help="Input .dae file")
    parser.add_argument("output", help="Output .stl file")
    parser.add_argument("--rotate-axis", choices=['x', 'y', 'z'], help="Rotation axis")
    parser.add_argument("--rotate-angle", type=float, default=0.0, help="Rotation angle in degrees")
    parser.add_argument("--pivot", type=float, nargs=3, default=[0, 0, 0], 
                        help="Pivot point for rotation (x y z) in mesh coordinates")
    parser.add_argument("--scale", type=float, default=1.0, help="Uniform scale factor")
    args = parser.parse_args()

    convert_dae_to_stl(args.input, args.output, 
                       rotate_axis=args.rotate_axis,
                       rotate_angle=args.rotate_angle,
                       pivot=args.pivot,
                       scale=args.scale)