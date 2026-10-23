import json
import os

TILESET_PATH = os.path.join(os.path.dirname(__file__), 'tileset.json')


def test_load_tile_base_walls_returns_dict():
    from tile_utils import load_tile_base_walls
    result = load_tile_base_walls(TILESET_PATH)
    assert isinstance(result, dict)


def test_load_tile_base_walls_includes_straight():
    from tile_utils import load_tile_base_walls
    result = load_tile_base_walls(TILESET_PATH)
    assert 'straight' in result


def test_load_tile_base_walls_skips_empty_wall_tiles():
    from tile_utils import load_tile_base_walls
    result = load_tile_base_walls(TILESET_PATH)
    with open(TILESET_PATH) as f:
        tiles = json.load(f)
    empty_tiles = [t['name'] for t in tiles if not t['walls']]
    for name in empty_tiles:
        assert name not in result, f'{name} has empty walls but was included'


def test_load_tile_base_walls_returns_wall_lists():
    from tile_utils import load_tile_base_walls
    result = load_tile_base_walls(TILESET_PATH)
    for name, walls in result.items():
        assert isinstance(walls, list), f'{name} walls should be a list'
        for wall in walls:
            assert isinstance(wall, list), f'wall in {name} should be a list'
            for coord in wall:
                assert len(coord) == 2, f'coord in {name} should be (x, y)'


def test_load_tile_base_walls_block_tile_has_full_boundary():
    from tile_utils import load_tile_base_walls
    result = load_tile_base_walls(TILESET_PATH)
    assert 'block' in result
    assert len(result['block']) == 1
