import argparse
import glob
import os
import shutil

from PIL import Image

import cairosvg


def convert_svg_to_png(input_path: str, output_path: str):
    if not os.path.exists(input_path):
        raise FileNotFoundError(f'Input file not found: {input_path}')
    with open(input_path, 'rb') as f:
        svg_data = f.read()
    try:
        png_data = cairosvg.svg2png(bytestring=svg_data)
    except Exception as e:
        raise ValueError(f'Failed to convert SVG to PNG: {e}')
    with open(output_path, 'wb') as f:
        f.write(png_data)
    img = Image.open(output_path).convert('L')
    img = img.point(lambda x: 255 if x > 127 else 0, mode='1')
    img = img.convert('L')
    img.save(output_path)


_GAZEBO_TEXTURES_REL = (
    '../../src/ag_map_gazebo/models/ground_plane/materials/textures')
GAZEBO_TEXTURES_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), _GAZEBO_TEXTURES_REL))


def convert_and_copy_to_gazebo(svg_path: str, map_name: str):
    """Convert SVG to PNG and copy to the Gazebo model textures directory."""
    png_path = svg_path.replace('.svg', '.png')
    convert_svg_to_png(svg_path, png_path)
    gazebo_path = os.path.join(GAZEBO_TEXTURES_DIR, map_name + '.png')
    os.makedirs(os.path.dirname(gazebo_path), exist_ok=True)
    shutil.copy2(png_path, gazebo_path)
    print(f'Copied {png_path} -> {gazebo_path}')


def convert_all_svgs(output_dir: str = 'worlds/maps'):
    svg_files = glob.glob(os.path.join(output_dir, '*_map.svg'))
    svg_files.sort()
    for svg_path in svg_files:
        png_path = svg_path.replace('.svg', '.png')
        print(f'Converting {svg_path} -> {png_path}')
        convert_svg_to_png(svg_path, png_path)


def main():
    parser = argparse.ArgumentParser(
        description='Convert SVG maps to grayscale PNGs')
    parser.add_argument('--input', type=str,
                        help='Path to single SVG file to convert')
    parser.add_argument('--all', action='store_true',
                        help='Convert all *_map.svg files')
    parser.add_argument('--output-dir', type=str, default='worlds/maps',
                        help='Directory containing SVGs')
    args = parser.parse_args()
    if args.input:
        output_path = args.input.replace('.svg', '.png')
        convert_svg_to_png(args.input, output_path)
        print(f'Converted {args.input} -> {output_path}')
    elif args.all:
        convert_all_svgs(args.output_dir)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
