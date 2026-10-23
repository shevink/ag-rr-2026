import argparse
import os

from convert_to_png import convert_and_copy_to_gazebo
from geometry_utils import rotate_walls, transform
from svg_renderer import SVGRenderer
from tile_utils import load_tile_base_walls

TILESET_PATH = os.path.join(os.path.dirname(__file__), 'tileset.json')


SKEW_GRID = [
    [('block', 0), ('block', 0), ('block', 0), ('block', 0),
     ('block', 0), ('block', 0), ('block', 0), ('block', 0)],
    [('block', 0), ('straight_skew', 0), ('cross', 0), ('straight_skew', 0),
     ('straight_skew', 0), ('cross', 0), ('straight_skew', 0), ('block', 0)],
    [('block', 0), ('block', 0), ('straight', 1), ('straight', 1),
     ('block', 0), ('straight', 1), ('straight', 1), ('block', 0)],
    [('block', 0), ('straight_skew', 0), ('cross', 0), ('junction', 2),
     ('block', 0), ('cross', 0), ('straight_skew', 0), ('block', 0)],
    [('block', 0), ('junction', 2), ('straight_skew', 0),
     ('straight_skew', 0), ('block', 0), ('block', 0), ('block', 0),
     ('block', 0)],
    [('block', 0), ('block', 0), ('block', 0), ('block', 0),
     ('block', 0), ('block', 0), ('block', 0), ('block', 0)],
]

GRID_ROWS = len(SKEW_GRID)
GRID_COLS = len(SKEW_GRID[0])


def generate_medium_map(seed=42, output_path='worlds/maps/medium_map.svg'):
    """Generate a medium-difficulty SVG map with skewed tiles."""
    tile_walls = load_tile_base_walls(TILESET_PATH)

    canvas_width = GRID_COLS * 100
    canvas_height = GRID_ROWS * 100
    renderer = SVGRenderer(
        width=canvas_width,
        height=canvas_height,
        viewbox=f'0 0 {GRID_COLS} {GRID_ROWS}'
    )

    for row_idx, row in enumerate(SKEW_GRID):
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
    """Parse CLI arguments and generate the medium map."""
    parser = argparse.ArgumentParser(
        description='Generate a medium map as SVG'
    )
    parser.add_argument(
        '--seed', type=int, default=42,
        help='Seed (maps are deterministic, accepted for API consistency)'
    )
    parser.add_argument(
        '--output', type=str, default='worlds/maps/medium_map.svg',
        help='Output SVG file path'
    )
    args = parser.parse_args()
    generate_medium_map(seed=args.seed, output_path=args.output)
    print(f'Medium map saved to {args.output}')
    convert_and_copy_to_gazebo(args.output, 'medium_map')


if __name__ == '__main__':
    main()
