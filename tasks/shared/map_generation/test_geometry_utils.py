import math


def test_rotate_walls_identity():
    from geometry_utils import rotate_walls
    walls = [[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]]
    result = rotate_walls(walls, 0)
    assert len(result) == 1
    for orig, rot in zip(walls[0], result[0]):
        assert abs(orig[0] - rot[0]) < 1e-6
        assert abs(orig[1] - rot[1]) < 1e-6


def test_rotate_walls_90():
    from geometry_utils import rotate_walls
    walls = [[(0.0, 0.0), (1.0, 0.0), (1.0, 0.25), (0.0, 0.25)]]
    result = rotate_walls(walls, 1)
    x, y = result[0][0]
    assert abs(x - 1.0) < 1e-6
    assert abs(y - 0.0) < 1e-6


def test_rotate_walls_180():
    from geometry_utils import rotate_walls
    walls = [[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]]
    result = rotate_walls(walls, 2)
    assert len(result) == 1
    assert abs(result[0][0][0] - 1.0) < 1e-6
    assert abs(result[0][0][1] - 1.0) < 1e-6
    assert abs(result[0][1][0] - 0.0) < 1e-6
    assert abs(result[0][1][1] - 1.0) < 1e-6


def test_rotate_walls_270():
    from geometry_utils import rotate_walls
    walls = [[(0.0, 0.0), (1.0, 0.0), (1.0, 0.25), (0.0, 0.25)]]
    result = rotate_walls(walls, 3)
    x, y = result[0][0]
    assert abs(x - 0.0) < 1e-6
    assert abs(y - 1.0) < 1e-6


def test_transform_translation_only():
    from geometry_utils import transform
    coords = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    result = transform(coords, 5.0, 10.0, 0.0)
    assert len(result) == 3
    assert abs(result[0][0] - 5.0) < 1e-6
    assert abs(result[0][1] - 10.0) < 1e-6
    assert abs(result[1][0] - 6.0) < 1e-6
    assert abs(result[1][1] - 10.0) < 1e-6


def test_transform_rotation_only():
    from geometry_utils import transform
    coords = [(1.0, 0.0)]
    result = transform(coords, 0.0, 0.0, math.pi / 2)
    assert abs(result[0][0]) < 1e-6
    assert abs(result[0][1] - 1.0) < 1e-6


def test_transform_combined():
    from geometry_utils import transform
    coords = [(1.0, 0.0)]
    result = transform(coords, 5.0, 10.0, math.pi / 2)
    assert abs(result[0][0] - 5.0) < 1e-6
    assert abs(result[0][1] - 11.0) < 1e-6


def test_transform_negative_translation():
    from geometry_utils import transform
    coords = [(0.0, 0.0)]
    result = transform(coords, -5.0, -3.0, 0.0)
    assert abs(result[0][0] + 5.0) < 1e-6
    assert abs(result[0][1] + 3.0) < 1e-6


def test_rotate_walls_empty():
    from geometry_utils import rotate_walls
    result = rotate_walls([], 1)
    assert result == []


def test_rotate_walls_multiple_walls():
    from geometry_utils import rotate_walls
    walls = [
        [(0.0, 0.0), (1.0, 0.0), (1.0, 0.25), (0.0, 0.25)],
        [(0.0, 0.75), (0.0, 1.0), (1.0, 1.0), (1.0, 0.75)]
    ]
    result = rotate_walls(walls, 1)
    assert len(result) == 2
    for wall in result:
        assert len(wall) == 4


def test_approximate_curve_segment_count():
    from geometry_utils import approximate_curve
    cps = [(0.0, 0.0), (0.5, 1.0), (1.0, 0.0)]
    result = approximate_curve(cps, 8)
    assert len(result) == 9


def test_approximate_curve_endpoints():
    from geometry_utils import approximate_curve
    cps = [(0.0, 0.0), (0.5, 1.0), (1.0, 0.0)]
    result = approximate_curve(cps, 8)
    assert abs(result[0][0]) < 1e-6
    assert abs(result[0][1]) < 1e-6
    assert abs(result[-1][0] - 1.0) < 1e-6
    assert abs(result[-1][1]) < 1e-6


def test_approximate_curve_four_cps():
    from geometry_utils import approximate_curve
    cps = [(0.0, 0.0), (0.33, 1.0), (0.66, 1.0), (1.0, 0.0)]
    result = approximate_curve(cps, 10)
    assert len(result) == 11


def test_create_star_intersection_3_roads():
    from geometry_utils import create_star_intersection
    result = create_star_intersection((0, 0), 3, 1.0)
    assert len(result) == 4


def test_create_star_intersection_5_roads():
    from geometry_utils import create_star_intersection
    result = create_star_intersection((0, 0), 5, 1.0)
    assert len(result) == 6


def test_create_roundabout_rings():
    from geometry_utils import create_roundabout
    result = create_roundabout((0, 0), 0.5, 1.0, 16)
    assert len(result) == 2


def test_create_roundabout_vertex_count():
    from geometry_utils import create_roundabout
    result = create_roundabout((0, 0), 0.5, 1.0, 16)
    assert len(result[0]) == 16
    assert len(result[1]) == 16
