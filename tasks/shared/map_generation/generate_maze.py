import argparse
import json
import os
import random
import warnings
from collections import deque

from convert_to_png import convert_and_copy_to_gazebo
from geometry_utils import rotate_walls
from svg_renderer import SVGRenderer


def _edge_mask(edges):
    mask = 0
    for i in range(4):
        if edges[i] == 'X':
            mask |= 1 << (3 - i)
    return mask


BASE_TILE_NAMES = frozenset({
    'straight', 'curve_90', 't_junction', 'cross', 'block', 'endcap'
})


def load_tileset(tileset_path):
    with open(tileset_path) as f:
        data = json.load(f)

    tiles_by_mask = {}
    for tile in data:
        name = tile.get('name', '')
        walls = tile.get('walls', [])
        edge_list = tile.get('edges', [])

        if name not in BASE_TILE_NAMES:
            continue
        if not walls or not edge_list:
            continue

        for rot_idx, edges in enumerate(edge_list):
            mask = _edge_mask(edges)
            extra_rot = 0
            if 'curve' in name:
                extra_rot = 1
            elif 'endcap' in name:
                extra_rot = 2
            rotation = (rot_idx + extra_rot) % 4
            if rotation == 0:
                rotated_walls = walls
            else:
                rotated_walls = rotate_walls(walls, rotation)
            tiles_by_mask[mask] = {
                'name': name,
                'walls': rotated_walls,
                'edges': edges,
                'rotation': rotation,
            }

    return tiles_by_mask


def generate_maze(seed, size, solution_edges=None):
    rng = random.Random(seed)
    walls_4 = [True, True, True, True]
    maze = [[walls_4[:] for _ in range(size)] for _ in range(size)]
    visited = [[False] * size for _ in range(size)]
    stack = [(0, 0)]
    visited[0][0] = True
    dx = [1, 0, -1, 0]
    dy = [0, 1, 0, -1]
    if solution_edges is None:
        solution_edges = []
    while stack:
        x, y = stack[-1]
        dirs = [0, 1, 2, 3]
        rng.shuffle(dirs)
        moved = False
        for d in dirs:
            nx, ny = x + dx[d], y + dy[d]
            if 0 <= nx < size and 0 <= ny < size and not visited[ny][nx]:
                maze[y][x][d] = False
                maze[ny][nx][(d + 2) % 4] = False
                visited[ny][nx] = True
                stack.append((nx, ny))
                solution_edges.append(((x, y), (nx, ny)))
                moved = True
                break
        if not moved:
            stack.pop()
    maze[0][0][3] = False
    maze[size - 1][size - 1][1] = False
    return maze


def add_crossings(grid, crossing_prob, rng, solution_edges=None):
    size = len(grid)
    for y in range(1, size - 1):
        for x in range(1, size - 1):
            open_count = sum(1 for d in range(4) if not grid[y][x][d])
            if open_count < 4 and rng.random() < crossing_prob:
                for d in range(4):
                    if grid[y][x][d]:
                        grid[y][x][d] = False
                        dx = 1 if d == 0 else -1 if d == 2 else 0
                        dy = 1 if d == 1 else -1 if d == 3 else 0
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < size and 0 <= ny < size:
                            grid[ny][nx][(d + 2) % 4] = False
                            if solution_edges is not None:
                                solution_edges.append(((x, y), (nx, ny)))


def find_matching_tile(walls, tiles_by_mask):
    mask = 0
    mask |= (1 if walls[0] else 0) << 3
    mask |= (1 if walls[1] else 0) << 2
    mask |= (1 if walls[2] else 0) << 1
    mask |= (1 if walls[3] else 0) << 0
    if mask in tiles_by_mask:
        return tiles_by_mask[mask]
    raise ValueError(f'No matching tile for wall config: {walls}')


TILE_SIZE = 100
TILESET_PATH = os.path.join(
    os.path.dirname(__file__), 'tileset.json'
)


def _verify_path(maze, start, end):
    size = len(maze)
    q = deque([start])
    visited = {start}
    while q:
        x, y = q.popleft()
        walls = maze[y][x]
        for d, (dx, dy) in enumerate([(1, 0), (0, 1), (-1, 0), (0, -1)]):
            if not walls[d]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < size and 0 <= ny < size:
                    if (nx, ny) not in visited:
                        if (nx, ny) == end:
                            return True
                        visited.add((nx, ny))
                        q.append((nx, ny))
    return False


def generate_maze_map(seed, size, crossing_prob, output_path,
                      max_attempts=100):
    rng = random.Random(seed)
    tiles = load_tileset(TILESET_PATH)
    for attempt in range(max_attempts):
        solution_edges = []
        grid = generate_maze(seed, size, solution_edges=solution_edges)
        add_crossings(grid, crossing_prob, rng, solution_edges=solution_edges)

        if _verify_path(grid, (0, 0), (size - 1, size - 1)):
            break
        msg = f'Seed {seed} produced unsolvable maze, retrying'
        warnings.warn(msg)
        seed = rng.randrange(2**31)
    else:
        raise RuntimeError(
            f'Failed to generate solvable maze after {max_attempts} attempts'
        )

    canvas_size = size * TILE_SIZE
    renderer = SVGRenderer(
        width=canvas_size,
        height=canvas_size,
        viewbox=f'0 0 {size} {size}'
    )
    for y in range(size):
        for x in range(size):
            walls = grid[y][x]
            try:
                tile_info = find_matching_tile(walls, tiles)
            except ValueError:
                tile_info = None
            if tile_info is None:
                continue
            for wall_poly in tile_info['walls']:
                global_pts = [
                    (px + x, py + y) for px, py in wall_poly
                ]
                renderer.add_polygon(
                    global_pts, stroke_width=0.02
                )

    # Start marker (white 'S') at top-left cell (0,0)
    renderer.add_text(0.5, 0.5, 'S', fill='#ffffff', font_size=0.4)

    # End marker (white 'E') at bottom-right cell (N-1,N-1)
    renderer.add_text(size - 0.5, size - 0.5, 'E', fill='#ffffff',
                      font_size=0.4)

    renderer.save(output_path)


def main():
    parser = argparse.ArgumentParser(
        description='Generate a maze map as SVG'
    )
    parser.add_argument(
        '--seed', type=int, default=None,
        help='Random seed for reproducible maze'
    )
    parser.add_argument(
        '--size', type=int, default=8,
        help='Grid size (N×N), default: 8'
    )
    parser.add_argument(
        '--crossing', type=float, default=0.2,
        help='Crossing probability, default: 0.2'
    )
    parser.add_argument(
        '--output', type=str, default='worlds/maps/maze_map.svg',
        help='Output SVG file path'
    )
    args = parser.parse_args()
    seed = args.seed if args.seed is not None else random.randrange(2**31)
    generate_maze_map(seed, args.size, args.crossing, args.output)
    print(f'Maze map saved to {args.output}')
    print(f'Seed: {seed}, Size: {args.size}x{args.size}, '
          f'Crossing prob: {args.crossing}')
    convert_and_copy_to_gazebo(args.output, 'maze_map')


if __name__ == '__main__':
    main()
