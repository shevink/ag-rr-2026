import argparse
import os

from convert_to_png import convert_and_copy_to_gazebo
from geometry_utils import rotate_walls, transform
from svg_renderer import SVGRenderer
from tile_utils import load_tile_base_walls

TILESET_PATH = os.path.join(os.path.dirname(__file__), 'tileset.json')


GRID = [
    [('block', 0), ('block', 0), ('block', 0), ('block', 0), ('block', 0), ('block', 0), ('block', 0), ('block', 0)],
    [('block', 0), ('straight', 0), ('cross', 0), ('straight', 0), ('straight_onewall', 0), ('straight_onewall', 0), ('straight_onewall', 0), ('block', 0)],
    [('block', 0), ('endcap', 2), ('t_junction', 1), ('block', 0), ('parkbucht_start', 0), ('parkbucht_middle', 0), ('parkbucht_end', 0), ('block', 0)],
    [('block', 0), ('block', 0), ('block', 0), ('block', 0), ('block', 0), ('block', 0), ('block', 0), ('block', 0)],
    [('block', 0), ('block', 0), ('block', 0), ('block', 0), ('block', 0), ('block', 0), ('block', 0), ('block', 0)],
]

GRID_ROWS = len(GRID)
GRID_COLS = len(GRID[0])


def generate_easy_map(seed=42, output_path='worlds/maps/easy_map.svg'):
    """Generate an easy-difficulty SVG map with basic tile road network."""
    tile_walls = load_tile_base_walls(TILESET_PATH)

    canvas_width = GRID_COLS * 100
    canvas_height = GRID_ROWS * 100
    renderer = SVGRenderer(
        width=canvas_width,
        height=canvas_height,
        viewbox=f'0 0 {GRID_COLS} {GRID_ROWS}'
    )

    for row_idx, row in enumerate(GRID):
        for col_idx, (tile_name, rotation) in enumerate(row):
            if tile_name not in tile_walls:
                continue
            walls = tile_walls[tile_name]
            if rotation > 0:
                walls = rotate_walls(walls, rotation)
            for wall_poly in walls:
                global_pts = transform(wall_poly, col_idx, row_idx, 0.0)
                renderer.add_polygon(global_pts, stroke_width=0.02)

    renderer.save(output_path)


def main():
    """Parse CLI arguments and generate the easy map."""
    parser = argparse.ArgumentParser(
        description='Generate an easy map as SVG'
    )
    parser.add_argument(
        '--seed', type=int, default=42,
        help='Seed (maps are deterministic, accepted for API consistency)'
    )
    parser.add_argument(
        '--output', type=str, default='worlds/maps/easy_map.svg',
        help='Output SVG file path'
    )
    args = parser.parse_args()
    generate_easy_map(seed=args.seed, output_path=args.output)
    print(f'Easy map saved to {args.output}')
    convert_and_copy_to_gazebo(args.output, 'easy_map')


if __name__ == '__main__':
    main()
