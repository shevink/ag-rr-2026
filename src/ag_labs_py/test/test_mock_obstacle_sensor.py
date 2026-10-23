"""Unit tests for mock_obstacle_sensor module."""

import math
from unittest.mock import MagicMock


def test_module_import():
    """Module should be importable."""
    import ag_labs_py.mock_obstacle_sensor
    assert hasattr(ag_labs_py.mock_obstacle_sensor, 'MockObstacleSensor')
    assert hasattr(ag_labs_py.mock_obstacle_sensor, 'main')


def test_obstacle_topics_from_config_includes_dynamic_and_static(tmp_path):
    """Config discovery should include every configured obstacle."""
    from ag_labs_py.mock_obstacle_sensor import _obstacle_topics_from_config

    config_path = tmp_path / 'config.yaml'
    config_path.write_text(
        '\n'.join(
            [
                'dynamic:',
                '  obstacle:',
                '    spawn_position: {x: 1, y: 1}',
                '  obstacle_two:',
                '    spawn_position: {x: 2, y: 2}',
                'static:',
                '  rock_one:',
                '    spawn_position: {x: 3, y: 3}',
            ]
        )
    )

    assert _obstacle_topics_from_config(str(config_path)) == [
        '/obstacle/pose_ground_truth',
        '/obstacle_two/pose_ground_truth',
        '/rock_one/pose_ground_truth',
    ]


def test_default_field_of_view_is_90_degrees():
    """Default FOV should be narrower than the old 120 degree setup."""
    from ag_labs_py.mock_obstacle_sensor import DEFAULT_FIELD_OF_VIEW

    assert math.isclose(DEFAULT_FIELD_OF_VIEW, math.pi / 2.0)


class TestNormalizeAngle:
    """Tests for the _normalize_angle static method."""

    def test_zero(self):
        """Zero should remain zero."""
        from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor
        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        result = sensor._normalize_angle(0.0)
        assert result == 0.0

    def test_positive_angle(self):
        """Positive angle within range should stay unchanged."""
        from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor
        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        result = sensor._normalize_angle(1.0)
        assert math.isclose(result, 1.0)

    def test_negative_angle(self):
        """Negative angle within range should stay unchanged."""
        from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor
        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        result = sensor._normalize_angle(-1.0)
        assert math.isclose(result, -1.0)

    def test_above_pi(self):
        """Angle above pi should wrap around."""
        from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor
        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        result = sensor._normalize_angle(math.pi + 0.5)
        assert math.isclose(result, -math.pi + 0.5, abs_tol=1e-9)

    def test_below_neg_pi(self):
        """Angle below -pi should wrap around."""
        from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor
        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        result = sensor._normalize_angle(-math.pi - 0.5)
        assert math.isclose(result, math.pi - 0.5, abs_tol=1e-9)

    def test_multiple_wraps(self):
        """Angle with multiple full rotations should wrap correctly."""
        from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor
        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        result = sensor._normalize_angle(3 * math.pi + 0.3)
        assert math.isclose(result, -math.pi + 0.3, abs_tol=1e-9)


class TestQuaternionToYaw:
    """Tests for the _quaternion_to_yaw static method."""

    def test_zero_yaw(self):
        """Quaternion with no rotation should give 0 yaw."""
        from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor
        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        q = MagicMock()
        q.w = 1.0
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        result = sensor._quaternion_to_yaw(q)
        assert math.isclose(result, 0.0, abs_tol=1e-9)

    def test_90_degree_yaw(self):
        """90 degree yaw about Z should be pi/2."""
        from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor
        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        q = MagicMock()
        q.w = math.cos(math.pi / 4)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(math.pi / 4)
        result = sensor._quaternion_to_yaw(q)
        assert math.isclose(result, math.pi / 2, abs_tol=1e-9)

    def test_180_degree_yaw(self):
        """180 degree yaw about Z should be pi."""
        from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor
        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        q = MagicMock()
        q.w = 0.0
        q.x = 0.0
        q.y = 0.0
        q.z = 1.0
        result = sensor._quaternion_to_yaw(q)
        assert math.isclose(result, math.pi, abs_tol=1e-9)
