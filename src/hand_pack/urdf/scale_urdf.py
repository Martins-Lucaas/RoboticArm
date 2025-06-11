#!/usr/bin/env python3
"""
scale_urdf.py

This script scales all <origin xyz="..."> attributes and <mesh scale="..."> attributes
in a URDF file by a given factor. It multiplies each origin coordinate by the factor
and sets each mesh scale to the factor uniformly.

Usage:
    ./scale_urdf.py --factor 0.1 input.urdf output.urdf
"""

import argparse
import re
import sys
import os

def scale_urdf(input_path: str, output_path: str, factor: float):
    # Patterns to match <origin> xyz attributes and <mesh> scale attributes
    origin_pattern = re.compile(r'(<origin[^>]*\sxyz=")([^"]+)(")')
    mesh_scale_pattern = re.compile(r'(<mesh[^>]*\s)scale="[^"]*"')

    # Read input file
    with open(input_path, 'r', encoding='utf-8') as infile:
        content = infile.read()

    # Scale origin xyz coordinates
    def scale_origin(match):
        prefix, coords_str, suffix = match.groups()
        coords = [float(c) for c in coords_str.split()]
        scaled_coords = [c * factor for c in coords]
        scaled_str = ' '.join(f'{c:.12f}' for c in scaled_coords)
        return f'{prefix}{scaled_str}{suffix}'

    content = origin_pattern.sub(scale_origin, content)

    # Set mesh scale uniformly
    def replace_mesh_scale(match):
        pre = match.group(1)
        return f'{pre}scale="{factor:.6f} {factor:.6f} {factor:.6f}"'

    content = mesh_scale_pattern.sub(replace_mesh_scale, content)

    # Write output file
    with open(output_path, 'w', encoding='utf-8') as outfile:
        outfile.write(content)

def main():
    parser = argparse.ArgumentParser(
        description='Scale all origin xyz and mesh scale values in a URDF file.'
    )
    parser.add_argument('input', help='Path to input URDF file')
    parser.add_argument('output', help='Path to output scaled URDF file')
    parser.add_argument(
        '-f', '--factor', type=float, required=True,
        help='Scale factor (e.g., 0.1 for 10%)'
    )
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f'Error: Input file "{args.input}" does not exist.', file=sys.stderr)
        sys.exit(1)

    try:
        scale_urdf(args.input, args.output, args.factor)
        print(f'Scaled URDF saved to "{args.output}".')
    except Exception as e:
        print(f'Error during processing: {e}', file=sys.stderr)
        sys.exit(2)

if __name__ == '__main__':
    main()