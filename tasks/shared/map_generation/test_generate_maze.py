import os

TILESET_PATH = os.path.join(os.path.dirname(__file__), 'tileset.json')


def test_load_tileset_contains_all_base_tiles():
    from generate_maze import load_tileset
    tiles = load_tileset(TILESET_PATH)
    names = {info['name'] for info in tiles.values()}
    for required in ['straight', 'curve_90', 't_junction', 'cross',
                     'block', 'endcap']:
        assert required in names, f'Missing tile: {required}'


def test_load_tileset_has_expected_entries():
    from generate_maze import load_tileset
    tiles = load_tileset(TILESET_PATH)
    assert len(tiles) > 0


def test_straight_horizontal_mask():
    from generate_maze import load_tileset
    tiles = load_tileset(TILESET_PATH)
    mask = next(
        mask for mask, info in tiles.items()
        if info['name'] == 'straight' and info['edges'] == ['O', 'X', 'O', 'X']
    )
    assert mask == 0b0101


def test_straight_vertical_mask():
    from generate_maze import load_tileset
    tiles = load_tileset(TILESET_PATH)
    mask = next(
        mask for mask, info in tiles.items()
        if info['name'] == 'straight' and info['edges'] == ['X', 'O', 'X', 'O']
    )
    assert mask == 0b1010


def test_block_mask():
    from generate_maze import load_tileset
    tiles = load_tileset(TILESET_PATH)
    mask = next(m for m, i in tiles.items() if i['name'] == 'block')
    assert mask == 0b1111


def test_endcap_mask():
    from generate_maze import load_tileset
    tiles = load_tileset(TILESET_PATH)
    mask = next(
        mask for mask, info in tiles.items()
        if info['name'] == 'endcap' and info['edges'] == ['O', 'X', 'X', 'X']
    )
    assert mask == 0b0111


def test_endcap_rot90_mask():
    from generate_maze import load_tileset
    tiles = load_tileset(TILESET_PATH)
    mask = next(
        mask for mask, info in tiles.items()
        if info['name'] == 'endcap' and info['edges'] == ['X', 'O', 'X', 'X']
    )
    assert mask == 0b1011


def test_cross_mask():
    from generate_maze import load_tileset
    tiles = load_tileset(TILESET_PATH)
    mask = next(m for m, i in tiles.items() if i['name'] == 'cross')
    assert mask == 0b0000


def test_t_junction_mask():
    from generate_maze import load_tileset
    tiles = load_tileset(TILESET_PATH)
    edges = ['O', 'O', 'O', 'X']
    mask = next(
        m for m, i in tiles.items()
        if i['name'] == 't_junction' and i['edges'] == edges
    )
    assert mask == 0b0001


def test_each_tile_has_expected_number_of_rotations():
    from generate_maze import load_tileset
    tiles = load_tileset(TILESET_PATH)
    counts = {}
    for info in tiles.values():
        counts[info['name']] = counts.get(info['name'], 0) + 1
    assert counts['straight'] == 2
    assert counts['block'] == 1
    assert counts['cross'] == 1
    assert counts['curve_90'] == 4
    assert counts['endcap'] == 4
    assert counts['t_junction'] == 4


def test_each_entry_has_walls():
    from generate_maze import load_tileset
    tiles = load_tileset(TILESET_PATH)
    for mask, info in tiles.items():
        assert len(info['walls']) > 0, (
            f'Tile {info["name"]} mask {mask} has no walls'
        )


def test_extension_tiles_have_no_masks():
    from generate_maze import load_tileset
    tiles = load_tileset(TILESET_PATH)
    names = {info['name'] for info in tiles.values()}
    for ext in ['straight_skew', 'curve_sharp', 'junction_skew', 'roundabout']:
        msg = f'Extension tile {ext} should not produce masks'
        assert ext not in names, msg


# --- Phase 2: DFS Maze Generation ---


def test_generate_maze_returns_grid_of_correct_size():
    from generate_maze import generate_maze
    grid = generate_maze(seed=42, size=4)
    assert len(grid) == 4
    assert all(len(row) == 4 for row in grid)


def test_generate_maze_8x8():
    from generate_maze import generate_maze
    grid = generate_maze(seed=42, size=8)
    assert len(grid) == 8
    assert all(len(row) == 8 for row in grid)


def test_generate_maze_all_cells_visited():
    from generate_maze import generate_maze
    grid = generate_maze(seed=42, size=6)
    for y in range(6):
        for x in range(6):
            # every cell must have at least one open passage
            cell = grid[y][x]
            assert not all(cell), f'Cell ({x},{y}) has all walls closed'


def _is_reachable(grid, size):
    visited = set()
    stack = [(0, 0)]
    while stack:
        x, y = stack.pop()
        if (x, y) in visited:
            continue
        visited.add((x, y))
        walls = grid[y][x]
        # walls order: [E, S, W, N] — False = passage open
        if not walls[3] and y > 0:        # N open
            stack.append((x, y - 1))
        if not walls[0] and x < size - 1:  # E open
            stack.append((x + 1, y))
        if not walls[1] and y < size - 1:  # S open
            stack.append((x, y + 1))
        if not walls[2] and x > 0:         # W open
            stack.append((x - 1, y))
    return len(visited) == size * size


def test_generate_maze_fully_connected():
    from generate_maze import generate_maze
    grid = generate_maze(seed=42, size=6)
    assert _is_reachable(grid, 6), 'Not all 36 cells are reachable'


def test_generate_maze_deterministic():
    from generate_maze import generate_maze
    grid1 = generate_maze(seed=42, size=6)
    grid2 = generate_maze(seed=42, size=6)
    assert grid1 == grid2


def test_generate_maze_start_cell_has_north_entrance():
    from generate_maze import generate_maze
    grid = generate_maze(seed=42, size=6)
    msg = 'Start cell N wall should be open (entrance)'
    assert grid[0][0][3] is False, msg


def test_generate_maze_end_cell_has_south_exit():
    from generate_maze import generate_maze
    grid = generate_maze(seed=42, size=6)
    msg = 'End cell S wall should be open (exit)'
    assert grid[5][5][1] is False, msg


def test_generate_maze_different_seeds_different():
    from generate_maze import generate_maze
    grid1 = generate_maze(seed=42, size=6)
    grid2 = generate_maze(seed=99, size=6)
    assert grid1 != grid2


# --- Phase 3: Cycle Generation (Crossings) ---


def test_add_crossings_zero_probability_preserves_maze():
    from generate_maze import generate_maze, add_crossings
    import random
    rng = random.Random(42)
    grid = generate_maze(seed=42, size=6)
    original = [row[:] for row in grid]
    add_crossings(grid, 0.0, rng)
    assert grid == original, 'crossing_prob=0.0 should not change the maze'


def test_add_crossings_one_probability_opens_walls():
    from generate_maze import generate_maze, add_crossings
    import random
    rng = random.Random(42)
    grid = generate_maze(seed=42, size=6)
    original_walls = sum(sum(cell) for row in grid for cell in row)
    add_crossings(grid, 1.0, rng)
    new_walls = sum(sum(cell) for row in grid for cell in row)
    assert new_walls < original_walls, 'crossing_prob=1.0 should open walls'


def test_add_crossings_does_not_open_boundary():
    from generate_maze import generate_maze, add_crossings
    import random
    rng = random.Random(42)
    grid = generate_maze(seed=42, size=8)
    add_crossings(grid, 1.0, rng)
    for y in range(1, 7):
        assert grid[y][0][2] is True, f'W wall of (0,{y}) should remain'
        assert grid[y][7][0] is True, f'E wall of (7,{y}) should remain'
    for x in range(1, 7):
        assert grid[0][x][3] is True, f'N wall of ({x},0) should remain'
        assert grid[7][x][1] is True, f'S wall of ({x},7) should remain'


def test_add_crossings_deterministic():
    from generate_maze import generate_maze, add_crossings
    import random
    rng1 = random.Random(42)
    rng2 = random.Random(42)
    grid1 = generate_maze(seed=42, size=6)
    grid2 = generate_maze(seed=42, size=6)
    add_crossings(grid1, 0.2, rng1)
    add_crossings(grid2, 0.2, rng2)
    assert grid1 == grid2


# --- Phase 4: Tile Matching ---


def test_find_matching_tile_block():
    from generate_maze import load_tileset, find_matching_tile
    tiles = load_tileset(TILESET_PATH)
    info = find_matching_tile([True, True, True, True], tiles)
    assert info['name'] == 'block'


def test_find_matching_tile_straight_vertical():
    from generate_maze import load_tileset, find_matching_tile
    tiles = load_tileset(TILESET_PATH)
    info = find_matching_tile([True, False, True, False], tiles)
    assert info['name'] == 'straight'


def test_find_matching_tile_straight_horizontal():
    from generate_maze import load_tileset, find_matching_tile
    tiles = load_tileset(TILESET_PATH)
    info = find_matching_tile([False, True, False, True], tiles)
    assert info['name'] == 'straight'


def test_find_matching_tile_cross():
    from generate_maze import load_tileset, find_matching_tile
    tiles = load_tileset(TILESET_PATH)
    info = find_matching_tile([False, False, False, False], tiles)
    assert info['name'] == 'cross'


def test_find_matching_tile_t_junction():
    from generate_maze import load_tileset, find_matching_tile
    tiles = load_tileset(TILESET_PATH)
    info = find_matching_tile([False, False, False, True], tiles)
    assert info['name'] == 't_junction'


def test_find_matching_tile_returns_info_with_walls():
    from generate_maze import load_tileset, find_matching_tile
    tiles = load_tileset(TILESET_PATH)
    info = find_matching_tile([True, True, True, True], tiles)
    assert 'name' in info
    assert 'walls' in info
    assert 'rotation' in info
    assert len(info['walls']) > 0


# --- Phase 5: Solution Graph & Path Verification ---


def test_generate_maze_records_solution_edges():
    from generate_maze import generate_maze
    edges = []
    generate_maze(seed=42, size=8, solution_edges=edges)
    assert len(edges) >= 63, (
        'Expected at least 63 edges (spanning tree of 64 nodes), '
        f'got {len(edges)}'
    )
    for (x1, y1), (x2, y2) in edges:
        dist = abs(x1 - x2) + abs(y1 - y2)
        assert dist == 1, (
            f'Edge ({x1},{y1})->({x2},{y2}) not adjacent'
        )


def test_add_crossings_records_solution_edges():
    from generate_maze import generate_maze, add_crossings
    import random
    edges = []
    maze = generate_maze(seed=42, size=8, solution_edges=edges)
    before = len(edges)
    add_crossings(maze, 1.0, random.Random(42), solution_edges=edges)
    assert len(edges) > before, 'Crossings should add more edges'


def test_verify_path_finds_path_in_valid_maze():
    from generate_maze import generate_maze, add_crossings, _verify_path
    import random
    grid = generate_maze(seed=42, size=8)
    add_crossings(grid, 0.2, random.Random(42))
    assert _verify_path(grid, (0, 0), (7, 7))


def test_verify_path_returns_false_for_impossible():
    from generate_maze import _verify_path
    size = 4
    maze = [[[True, True, True, True] for _ in range(size)]
            for _ in range(size)]
    maze[0][0][3] = False  # entrance
    maze[size - 1][size - 1][1] = False  # exit
    assert not _verify_path(maze, (0, 0), (size - 1, size - 1))


def test_generate_maze_map_guarantees_solvable():
    from generate_maze import generate_maze_map
    import tempfile
    import os
    for seed in range(20):
        f = tempfile.NamedTemporaryFile(suffix='.svg', delete=False, mode='w')
        generate_maze_map(seed=seed, size=8,
                          crossing_prob=0.2, output_path=f.name)
        os.unlink(f.name)


# --- Phase 6: Integration & CLI ---


def _gen_svg(seed=42, size=4, crossing=0.0):
    from generate_maze import generate_maze_map
    import tempfile
    import os
    f = tempfile.NamedTemporaryFile(suffix='.svg', delete=False, mode='w')
    generate_maze_map(seed=seed, size=size,
                      crossing_prob=crossing, output_path=f.name)
    f.flush()
    content = open(f.name).read()
    os.unlink(f.name)
    return content


def test_generate_maze_map_creates_svg():
    content = _gen_svg()
    assert '<?xml' in content
    assert '<svg' in content
    assert '</svg>' in content


def test_generate_maze_map_has_black_background():
    content = _gen_svg()
    assert 'fill="#000000"' in content


def test_generate_maze_map_has_obstacle_polygons():
    content = _gen_svg()
    assert 'fill="#0a0a0a"' in content
    assert 'stroke="#ffffff"' in content


def test_generate_maze_map_has_start_marker():
    content = _gen_svg()
    assert 'fill="#ffffff"' in content and 'S' in content


def test_generate_maze_map_has_end_marker():
    content = _gen_svg()
    assert 'fill="#ffffff"' in content and 'E' in content


def test_generate_maze_map_deterministic():
    a = _gen_svg(seed=42)
    b = _gen_svg(seed=42)
    assert a == b


def test_generate_maze_map_larger_size():
    content = _gen_svg(size=12, crossing=0.2)
    assert 'fill="#0a0a0a"' in content
