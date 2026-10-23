"""Unit tests for obstacle_geometry module."""

import math

from ag_labs_py.obstacle_geometry import (
    compute_corners,
    sort_counter_clockwise,
    world_to_body,
)


class TestComputeCorners:
    """Tests for compute_corners function."""

    def test_centered_no_rotation(self):
        """Corners at origin with no rotation should align with axes."""
        ox, oy, oyaw, half = 0.0, 0.0, 0.0, 0.5
        corners = compute_corners(ox, oy, oyaw, half)
        assert len(corners) == 4
        expected = {(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)}
        assert set(corners) == expected

    def test_centered_with_rotation(self):
        """Corners at origin with 45deg rotation should be rotated."""
        ox, oy, oyaw, half = 0.0, 0.0, math.pi / 4, 0.5
        corners = compute_corners(ox, oy, oyaw, half)
        assert len(corners) == 4
        cos45 = math.cos(math.pi / 4)
        sin45 = math.sin(math.pi / 4)
        expected = {
            (-half * cos45 + half * sin45, -half * sin45 - half * cos45),
            (half * cos45 + half * sin45, half * sin45 - half * cos45),
            (half * cos45 - half * sin45, half * sin45 + half * cos45),
            (-half * cos45 - half * sin45, -half * sin45 + half * cos45),
        }
        for c in corners:
            assert any(
                math.isclose(c[0], e[0], abs_tol=1e-9)
                and math.isclose(c[1], e[1], abs_tol=1e-9)
                for e in expected
            )

    def test_offset_position(self):
        """Corners at offset position should be centered around (ox, oy)."""
        ox, oy, oyaw, half = 5.0, -3.0, 0.0, 0.5
        corners = compute_corners(ox, oy, oyaw, half)
        assert len(corners) == 4
        for cx, cy in corners:
            assert (
                math.isclose(cx - ox, 0.5, abs_tol=1e-9)
                or math.isclose(cx - ox, -0.5, abs_tol=1e-9)
            )
            assert (
                math.isclose(cy - oy, 0.5, abs_tol=1e-9)
                or math.isclose(cy - oy, -0.5, abs_tol=1e-9)
            )

    def test_zero_size(self):
        """Zero-size obstacle should have all corners at center."""
        corners = compute_corners(0.0, 0.0, 0.0, 0.0)
        assert len(corners) == 4
        assert all(c == (0.0, 0.0) for c in corners)

    def test_negative_half_size(self):
        """Negative half_size should still produce 4 corners."""
        corners = compute_corners(0.0, 0.0, 0.0, -0.3)
        assert len(corners) == 4


class TestSortCounterClockwise:
    """Tests for sort_counter_clockwise function."""

    def test_already_ccw(self):
        """Already CCW order should remain unchanged."""
        points = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
        result = sort_counter_clockwise(points)
        atan2_values = [math.atan2(p[1], p[0]) for p in result]
        assert atan2_values == sorted(atan2_values)

    def test_reversed_order(self):
        """Reversed input should be sorted to CCW order."""
        points = [(0.0, -1.0), (-1.0, 0.0), (0.0, 1.0), (1.0, 0.0)]
        result = sort_counter_clockwise(points)
        assert len(result) == 4
        atan2_values = [math.atan2(p[1], p[0]) for p in result]
        assert atan2_values == sorted(atan2_values)

    def test_single_point(self):
        """Single point should return itself."""
        points = [(3.0, 4.0)]
        result = sort_counter_clockwise(points)
        assert result == [(3.0, 4.0)]

    def test_two_points(self):
        """Two points should return both."""
        points = [(0.0, 1.0), (1.0, 0.0)]
        result = sort_counter_clockwise(points)
        assert len(result) == 2

    def test_collinear_points(self):
        """Collinear points should all be returned."""
        points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        result = sort_counter_clockwise(points)
        assert len(result) == 3


class TestWorldToBody:
    """Tests for world_to_body function."""

    def test_robot_at_origin_no_rotation(self):
        """Robot at origin facing +x should give same coordinates."""
        point = (3.0, 4.0)
        result = world_to_body(point, 0.0, 0.0, 0.0)
        assert result == (3.0, 4.0)

    def test_robot_at_origin_with_yaw(self):
        """Robot rotated 90deg should rotate point -90deg in body frame."""
        point = (1.0, 0.0)
        result = world_to_body(point, 0.0, 0.0, math.pi / 2)
        assert math.isclose(result[0], 0.0, abs_tol=1e-9)
        assert math.isclose(result[1], -1.0, abs_tol=1e-9)

    def test_robot_offset_no_rotation(self):
        """Robot offset without rotation should subtract offset."""
        point = (5.0, 5.0)
        result = world_to_body(point, 3.0, 3.0, 0.0)
        assert result == (2.0, 2.0)

    def test_robot_offset_with_rotation(self):
        """Robot offset with 180deg rotation should negate relative vector."""
        point = (2.0, 0.0)
        result = world_to_body(point, 1.0, 0.0, math.pi)
        assert math.isclose(result[0], -1.0, abs_tol=1e-9)
        assert math.isclose(result[1], 0.0, abs_tol=1e-9)

    def test_same_position(self):
        """Point at robot position should give (0, 0)."""
        point = (1.0, 1.0)
        result = world_to_body(point, 1.0, 1.0, 0.0)
        assert result == (0.0, 0.0)
