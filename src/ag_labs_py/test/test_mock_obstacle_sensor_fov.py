"""Unit tests for mock_obstacle_sensor FOV marker frame assignment."""

from unittest.mock import MagicMock


class TestFOVMarkers:
    """Tests for _make_fov_markers frame_id and body-frame correctness."""

    def test_fov_markers_use_source_frame(self):
        """All FOV markers must use self.source_frame as their frame_id."""
        from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor
        from rclpy.time import Time

        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        sensor.source_frame = 'racer/base_footprint'
        sensor.detection_range = 2.0
        sensor.field_of_view = 1.57

        stamp = MagicMock(spec=Time)
        markers = sensor._make_fov_markers(0.785, stamp, iter(range(100)).__next__)

        assert len(markers) > 0, 'Expected at least one FOV marker'
        for m in markers:
            assert m.header.frame_id == sensor.source_frame, (
                f'FOV marker has frame_id={m.header.frame_id}, '
                f'expected {sensor.source_frame}'
            )

    def test_fov_markers_body_frame_points_not_transformed(self):
        """FOV marker points must be in body frame (not transformed to world)."""
        from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor
        from rclpy.time import Time

        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        sensor.source_frame = 'racer/base_footprint'
        sensor.detection_range = 2.0
        sensor.field_of_view = 1.57

        stamp = MagicMock(spec=Time)
        markers = sensor._make_fov_markers(0.785, stamp, iter(range(100)).__next__)

        for m in markers:
            for p in m.points:
                # In body frame, points should not exceed detection_range + margin
                dist = (p.x ** 2 + p.y ** 2) ** 0.5
                assert dist <= sensor.detection_range * 1.1, (
                    f'Point ({p.x:.2f}, {p.y:.2f}) at distance {dist:.2f} '
                    f'exceeds detection_range {sensor.detection_range} — '
                    f'points may have been transformed to world frame'
                )

    def test_no_body_to_world_in_fov_source(self):
        """The _make_fov_markers method source must not contain _body_to_world."""
        import inspect
        import ag_labs_py.mock_obstacle_sensor as mod

        source = inspect.getsource(mod.MockObstacleSensor._make_fov_markers)
        assert '_body_to_world' not in source, (
            '_make_fov_markers still calls _body_to_world'
        )

    def test_fov_markers_not_empty(self):
        """_make_fov_markers must return markers with non-empty points."""
        from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor
        from rclpy.time import Time

        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        sensor.source_frame = 'racer/base_footprint'
        sensor.detection_range = 2.0
        sensor.field_of_view = 1.57

        stamp = MagicMock(spec=Time)
        markers = sensor._make_fov_markers(0.785, stamp, iter(range(100)).__next__)

        for m in markers:
            assert len(m.points) > 0, f'FOV marker {m.ns}/{m.id} has no points'
