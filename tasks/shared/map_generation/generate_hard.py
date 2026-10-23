import argparse
import os

from convert_to_png import convert_and_copy_to_gazebo
from geometry_utils import rotate_walls, transform
from svg_renderer import SVGRenderer
from tile_utils import load_tile_base_walls

TILESET_PATH = os.path.join(os.path.dirname(__file__), 'tileset.json')

R = ('block', 0)
S0 = ('straight', 0)
S1 = ('straight', 1)
C0 = ('curve_90', 0)
C1 = ('curve_90', 1)
C2 = ('curve_90', 2)
C3 = ('curve_90', 3)
TJ0 = ('t_junction', 0)
TJ1 = ('t_junction', 1)
TJ2 = ('t_junction', 2)
TJ3 = ('t_junction', 3)
EP0 = ('endcap', 0)
EP1 = ('endcap', 1)
EP2 = ('endcap', 2)
EP3 = ('endcap', 3)
CR = ('cross', 0)

ROUNDABOUT_TEMPLATE = [
    [R, S1, R],
    [S0, ('roundabout_center', 0), S0],
    [R, S1, R],
]

STAR_TEMPLATE = [
    [R, S1, R],
    [S0, ('star_center', 0), S0],
    [R, S1, R],
]


def _stamp_template(grid, template, top_row, left_col):
    for r, row in enumerate(template):
        for c, tile in enumerate(row):
            grid[top_row + r][left_col + c] = tile


# 10 rows x 10 cols — road network with two roundabouts, one star,
# and lots of varied tiles (curves, T-junctions, endcaps, crosses)
GRID = [
    # Row 0 — top access to features
    [R, R, S1, R, R, R, R, S1, R, R],
    # Row 1 — stamp roundabout1 at (1,1), star at (1,6)
    [R, R, R, R, R, R, R, R, R, R],
    # Row 2 — horizontal road + cross + curve
    [S0, S0, R, S0, CR, S0, R, S0, C0, C3],
    # Row 3 — mixed vertical + branches
    [S1, S0, R, C2, S1, R, S0, EP3, TJ0, S1],
    # Row 4 — more varieties
    [C1, S0, TJ2, TJ0, TJ2, S0, TJ1, S1, EP2, C0],
    # Row 5 — horizontal + curves
    [S0, S0, C3, TJ2, S0, C3, C1, TJ2, C3, R],
    # Row 6 — stamp roundabout2 at (6,1)
    [R, R, R, R, R, C1, S0, TJ0, C0, R],
    # Row 7
    [S0, S0, R, C1, S0, TJ0, S0, TJ1, R, R],
    # Row 8
    [R, TJ2, R, S0, CR, TJ2, C3, C1, S0, S0],
    # Row 9
    [R, R, C1, S0, TJ2, R, S1, R, R, R],
]

_stamp_template(GRID, ROUNDABOUT_TEMPLATE, 1, 1)
_stamp_template(GRID, STAR_TEMPLATE, 1, 6)
_stamp_template(GRID, ROUNDABOUT_TEMPLATE, 6, 1)

GRID_ROWS = len(GRID)
GRID_COLS = len(GRID[0])


def generate_hard_map(seed=42, output_path='worlds/maps/hard_map.svg'):
    """Generate a hard SVG map with roundabout and star intersection."""
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
    """Parse CLI arguments and generate the hard map."""
    parser = argparse.ArgumentParser(
        description='Generate a hard map as SVG'
    )
    parser.add_argument(
        '--seed', type=int, default=42,
        help='Seed (maps are deterministic, accepted for API consistency)'
    )
    parser.add_argument(
        '--output', type=str, default='worlds/maps/hard_map.svg',
        help='Output SVG file path'
    )
    args = parser.parse_args()
    generate_hard_map(seed=args.seed, output_path=args.output)
    print(f'Hard map saved to {args.output}')
    convert_and_copy_to_gazebo(args.output, 'hard_map')


if __name__ == '__main__':
    main()
