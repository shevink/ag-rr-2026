import os
import re
import tempfile


def _gen_hard_svg(seed=42):
    from generate_hard import generate_hard_map
    f = tempfile.NamedTemporaryFile(suffix='.svg', delete=False, mode='w')
    generate_hard_map(seed=seed, output_path=f.name)
    f.flush()
    content = open(f.name).read()
    os.unlink(f.name)
    return content


def test_generate_hard_creates_svg():
    content = _gen_hard_svg()
    assert '<?xml' in content
    assert '<svg' in content
    assert '</svg>' in content


def test_generate_hard_has_black_background():
    content = _gen_hard_svg()
    assert 'fill="#000000"' in content


def test_generate_hard_has_obstacle_polygons():
    content = _gen_hard_svg()
    assert 'fill="#0a0a0a"' in content
    assert 'stroke="#ffffff"' in content


def test_generate_hard_has_roundabout():
    content = _gen_hard_svg()
    # Check a point from roundabout_center's outer ring at grid pos (2,2)
    pt_x = 2.50
    pt_y = 2.30
    found = any(
        abs(float(x) - pt_x) < 0.01 and abs(float(y) - pt_y) < 0.01
        for coord in re.findall(r'[\d.]+,[\d.\-]+', content)
        for x, y in [coord.split(',')]
    )
    msg = f'Roundabout wall ({pt_x},{pt_y}) not found'
    assert found, msg


def test_generate_hard_has_roundabout_inner_ring():
    content = _gen_hard_svg()
    inner_x = 2.70
    inner_y = 2.50
    found = any(
        abs(float(x) - inner_x) < 0.01 and abs(float(y) - inner_y) < 0.01
        for coord in re.findall(r'[\d.]+,[\d.\-]+', content)
        for x, y in [coord.split(',')]
    )
    msg = f'Inner ring coord ({inner_x},{inner_y}) not found in SVG'
    assert found, msg


def test_generate_hard_deterministic():
    a = _gen_hard_svg(seed=42)
    b = _gen_hard_svg(seed=42)
    assert a == b


# --- Phase 3 Red Phase tests: grid layout ---

def test_roundabout_has_center_tile():
    from generate_hard import GRID
    assert GRID[2][2][0] == 'roundabout_center', (
        f'Roundabout center is {GRID[2][2][0]}, expected roundabout_center')


def test_roundabout_has_four_access_tiles():
    from generate_hard import GRID
    expected = {
        (2, 0): 'straight',
        (0, 2): 'straight',
        (3, 2): 'straight',
        (2, 3): 'straight',
    }
    for (col, row), expected_tile in expected.items():
        tile_name, _ = GRID[row][col]
        msg = (f'Access ({col},{row}) is {tile_name},'
               f' expected {expected_tile}')
        assert tile_name == expected_tile, msg


def test_star_has_center_tile():
    from generate_hard import GRID
    # Star stamped at (1,6) in the new 10x10 grid layout
    assert GRID[2][7][0] == 'star_center', (
        f'Star center is {GRID[2][7][0]}, expected star_center')


def test_star_has_access_tiles():
    from generate_hard import GRID
    # Star access tiles at (top, left, right, bottom) relative to (1,6)
    assert GRID[1][7] == ('straight', 1), 'Star top access missing'
    assert GRID[2][6] == ('straight', 0), 'Star left access missing'
    assert GRID[2][8] == ('straight', 0), 'Star right access missing'
    assert GRID[3][7] == ('straight', 1), 'Star bottom access missing'


def test_connecting_road_tiles_exist():
    from generate_hard import GRID
    # Grid was restructured from 8x10 to 10x10
    # Roundabout 1 at (1,1) center at GRID[2][2]
    assert GRID[2][2][0] == 'roundabout_center', (
        f'Roundabout 1 center is {GRID[2][2][0]}')
    # Roundabout 2 at (6,1) center at GRID[7][2]
    assert GRID[7][2][0] == 'roundabout_center', (
        f'Roundabout 2 center is {GRID[7][2][0]}')


# --- Phase 4 Red Phase test: docstrings ---

def test_generator_functions_have_docstrings():
    from generate_hard import generate_hard_map, main as hard_main
    assert generate_hard_map.__doc__, 'generate_hard_map missing docstring'
    assert hard_main.__doc__, 'generate_hard main() missing docstring'

    from generate_easy import generate_easy_map, main as easy_main
    assert generate_easy_map.__doc__, 'generate_easy_map missing docstring'
    assert easy_main.__doc__, 'generate_easy main() missing docstring'

    from generate_medium import generate_medium_map, main as med_main
    assert generate_medium_map.__doc__, 'generate_medium_map missing docstring'
    assert med_main.__doc__, 'generate_medium main() missing docstring'

    from tile_utils import load_tile_base_walls
    msg = 'load_tile_base_walls missing docstring'
    assert load_tile_base_walls.__doc__, msg


def test_generate_hard_has_output_path():
    from generate_hard import generate_hard_map
    f = tempfile.NamedTemporaryFile(suffix='.svg', delete=False, mode='w')
    output = f.name
    f.close()
    generate_hard_map(seed=42, output_path=output)
    assert os.path.exists(output)
    content = open(output).read()
    assert len(content) > 0
    os.unlink(output)
