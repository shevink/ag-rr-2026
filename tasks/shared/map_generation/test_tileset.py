import json
import os

TILESET_PATH = os.path.join(os.path.dirname(__file__), 'tileset.json')


def test_tileset_loads():
    with open(TILESET_PATH) as f:
        tiles = json.load(f)
    assert len(tiles) >= 6


def test_all_base_tiles_present():
    with open(TILESET_PATH) as f:
        tiles = json.load(f)
    names = [t['name'] for t in tiles]
    for required in ['straight', 'curve_90', 't_junction', 'cross',
                     'block', 'endcap']:
        assert required in names, f'Missing tile: {required}'


def test_tiles_have_required_fields():
    with open(TILESET_PATH) as f:
        tiles = json.load(f)
    for tile in tiles:
        assert 'name' in tile
        assert 'walls' in tile
        assert 'edges' in tile


def test_edge_definitions_valid():
    with open(TILESET_PATH) as f:
        tiles = json.load(f)
    for tile in tiles:
        if not tile['edges']:
            continue
        for edge in tile['edges']:
            for e in edge:
                assert e in ('O', 'X'), f"Invalid edge '{e}' in {tile['name']}"


def test_wall_coordinates_valid():
    with open(TILESET_PATH) as f:
        tiles = json.load(f)
    for tile in tiles:
        if not tile['walls']:
            continue
        for wall in tile['walls']:
            for coord in wall:
                assert len(coord) == 2
                x, y = coord
                assert 0.0 <= x <= 1.0, f"x={x} out of range in {tile['name']}"
                assert 0.0 <= y <= 1.0, f"y={y} out of range in {tile['name']}"


def _load_tileset():
    import os
    path = os.path.join(os.path.dirname(__file__), 'tileset.json')
    with open(path) as f:
        return json.load(f)


def test_curve_sharp_exists():
    tiles = _load_tileset()
    names = [t['name'] for t in tiles]
    assert 'curve_sharp' in names


def test_curve_sharp_has_non_empty_walls():
    tiles = _load_tileset()
    for tile in tiles:
        if tile['name'] == 'curve_sharp':
            assert len(tile['walls']) > 0
            break


def test_straight_skew_exists():
    tiles = _load_tileset()
    names = [t['name'] for t in tiles]
    assert 'straight_skew' in names


def test_straight_skew_has_walls():
    tiles = _load_tileset()
    for tile in tiles:
        if tile['name'] == 'straight_skew':
            assert len(tile['walls']) == 2
            for wall in tile['walls']:
                assert len(wall) >= 6, (
                    f'straight_skew wall should have >=6 points for sine'
                    f' curve approx, got {len(wall)}')
            break


def test_straight_skew_walls_not_linear():
    tiles = _load_tileset()
    for tile in tiles:
        if tile['name'] == 'straight_skew':
            for wall in tile['walls']:
                y_vals = [pt[1] for pt in wall]
                assert max(y_vals) - min(y_vals) > 0.1, (
                    'straight_skew wall should have vertical variation')
            break


def test_curve_sharp_walls_have_four_points():
    tiles = _load_tileset()
    for tile in tiles:
        if tile['name'] == 'curve_sharp':
            assert len(tile['walls']) > 0, 'curve_sharp should have walls'
            for wall in tile['walls']:
                assert len(wall) == 4, (
                    f'curve_sharp wall should have 4 points, got {len(wall)}')
            break
