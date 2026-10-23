import json
import math
import os
import tempfile

TILESET_PATH = os.path.join(os.path.dirname(__file__), 'tileset.json')


def test_tile_rotate_render():
    from svg_renderer import SVGRenderer
    from geometry_utils import rotate_walls

    with open(TILESET_PATH) as f:
        tiles = json.load(f)
    straight = next(t for t in tiles if t['name'] == 'straight')

    r = SVGRenderer(width=200, height=200, viewbox='0 0 2 2')
    for wall in straight['walls']:
        rotated = rotate_walls([wall], 0)[0]
        r.add_polygon(rotated)
    with tempfile.NamedTemporaryFile(suffix='.svg', delete=False,
                                     mode='w') as f:
        r.save(f.name)
        f.flush()
        with open(f.name) as f2:
            content = f2.read()
            assert 'fill="#0a0a0a"' in content
            assert 'stroke="#ffffff"' in content
        os.unlink(f.name)


def test_transform_tile_to_grid():
    from svg_renderer import SVGRenderer
    from geometry_utils import transform, rotate_walls

    with open(TILESET_PATH) as f:
        tiles = json.load(f)
    cross = next(t for t in tiles if t['name'] == 'cross')

    r = SVGRenderer(width=400, height=400, viewbox='0 0 4 4')
    gx, gy = 1.0, 1.0
    for wall in cross['walls']:
        rotated = rotate_walls([wall], 0)[0]
        global_coords = transform(rotated, gx, gy, 0.0)
        r.add_polygon(global_coords)
    with tempfile.NamedTemporaryFile(suffix='.svg', delete=False,
                                     mode='w') as f:
        r.save(f.name)
        f.flush()
        with open(f.name) as f2:
            content = f2.read()
            assert '<path' in content
            assert 'M 1' in content or 'M 1.' in content
        os.unlink(f.name)


def test_end_to_end_minimal_svg():
    from svg_renderer import SVGRenderer, OBSTACLE_FILL, \
        OBSTACLE_STROKE, BACKGROUND
    from geometry_utils import transform, rotate_walls

    with open(TILESET_PATH) as f:
        tiles = json.load(f)
    straight = next(t for t in tiles if t['name'] == 'straight')

    r = SVGRenderer(width=300, height=300, viewbox='0 0 3 3')
    for row in range(3):
        for col in range(3):
            gx, gy = col * 1.0, row * 1.0
            for wall in straight['walls']:
                rotated = rotate_walls([wall], 0)[0]
                coords = transform(rotated, gx, gy, 0.0)
                r.add_polygon(coords)

    with tempfile.NamedTemporaryFile(suffix='.svg', delete=False,
                                     mode='w') as f:
        r.save(f.name)
        f.flush()
        with open(f.name) as f2:
            content = f2.read()
            assert OBSTACLE_FILL in content
            assert OBSTACLE_STROKE in content
            assert BACKGROUND in content
            assert 'xmlns="http://www.w3.org/2000/svg"' in content
        os.unlink(f.name)


def test_rotated_tile_transform():
    from geometry_utils import rotate_walls, transform

    with open(TILESET_PATH) as f:
        tiles = json.load(f)
    endcap = next(t for t in tiles if t['name'] == 'endcap')

    for rot in range(4):
        rotated = rotate_walls(endcap['walls'], rot)
        transformed = transform(rotated[0], 5.0, 5.0, math.pi / 4)
        assert len(transformed) == 4
        for x, y in transformed:
            assert x > 0
            assert y > 0
