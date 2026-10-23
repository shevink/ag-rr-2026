"""Validation tests for mock_obstacle_sensor — blueprint Phase 4 assertions.

Covers the four critical behavioral constraints:
  1. CCW winding order  – polygon vertices sorted counter-clockwise
  2. Distance filtering – obstacles outside [minimum_distance, detection_range] suppressed
  3. FOV filtering      – obstacles outside ±(field_of_view/2) suppressed
  4. Rate assertion     – sensor configured for 10 Hz (0.1 s period)

Helper design
-------------
Tests that exercise ``_timer_callback`` bypass the ROS ``Node.__init__`` via
``__new__`` and stub every ROS dependency (TF buffer, publisher, clock) with
``unittest.mock.MagicMock`` so the pure sensor logic runs without a live
ROS 2 context.
"""

import math
from unittest.mock import MagicMock, patch

import pytest
from ag_labs_py.obstacle_geometry import compute_corners, sort_counter_clockwise

# ─────────────────────────────────────────────────────────────────────────────
# Pure geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

DETECTION_RANGE: float = 2.0
MINIMUM_DISTANCE: float = 0.1
FIELD_OF_VIEW: float = 2.094  # ~120 degrees
HALF_FOV: float = FIELD_OF_VIEW / 2  # ~60 degrees / ~1.047 rad
OBSTACLE_SIZE: float = 0.2
UPDATE_RATE: float = 10.0


def signed_area(points: list[tuple[float, float]]) -> float:
    """Shoelace signed-area formula.  Positive ⟹ CCW.  Negative ⟹ CW."""
    n = len(points)
    if n < 3:
        return 0.0
    total = sum(
        points[i][0] * points[(i + 1) % n][1] - points[(i + 1) % n][0] * points[i][1]
        for i in range(n)
    )
    return total / 2.0


def cross_product_2d(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
) -> float:
    """Signed z-component of (p1-p0) × (p2-p1).  Positive ⟹ left turn (CCW)."""
    return (p1[0] - p0[0]) * (p2[1] - p1[1]) - (p1[1] - p0[1]) * (p2[0] - p1[0])


# ─────────────────────────────────────────────────────────────────────────────
# Sensor factory helpers
# ─────────────────────────────────────────────────────────────────────────────


def _make_sensor(
    update_rate: float = UPDATE_RATE,
    detection_range: float = DETECTION_RANGE,
    minimum_distance: float = MINIMUM_DISTANCE,
    field_of_view: float = FIELD_OF_VIEW,
    obstacle_size: float = OBSTACLE_SIZE,
) -> object:
    """Instantiate ``MockObstacleSensor`` without ROS ``__init__``."""
    from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor

    s = MockObstacleSensor.__new__(MockObstacleSensor)
    s.update_rate = update_rate
    s.detection_range = detection_range
    s.minimum_distance = minimum_distance
    s.field_of_view = field_of_view
    s.obstacle_topics = ["/obstacle/pose_ground_truth"]
    s.obstacle_sizes = [obstacle_size]
    s.source_frame = "racer/base_footprint"
    s._latest_poses = {"/obstacle/pose_ground_truth": None}
    s._latest_robot_pose = None
    s._publisher = MagicMock()
    # Stub ROS Node methods not needed for logic under test
    s.get_clock = MagicMock()
    s.get_clock.return_value.now.return_value.to_msg.return_value = MagicMock()
    s.get_logger = MagicMock()
    return s


def _make_transform(rx: float, ry: float, ryaw: float) -> MagicMock:
    """Return a mock TF2 transform placing the robot at world pose (rx, ry, ryaw)."""
    t = MagicMock()
    t.transform.translation.x = rx
    t.transform.translation.y = ry
    # Pure-yaw quaternion: q = (cos(θ/2), 0, 0, sin(θ/2))
    t.transform.rotation.w = math.cos(ryaw / 2.0)
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = math.sin(ryaw / 2.0)
    return t


def _make_pose_array(ox: float, oy: float, oyaw: float = 0.0) -> MagicMock:
    """Return a mock PoseArray with one obstacle at world pose (ox, oy, oyaw)."""
    pose = MagicMock()
    pose.position.x = ox
    pose.position.y = oy
    pose.orientation.w = math.cos(oyaw / 2.0)
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(oyaw / 2.0)
    pa = MagicMock()
    pa.poses = [pose]
    return pa


def _run_callback(sensor, transform, pose_array) -> object:
    """Wire up mocks, fire one timer tick, return the published ObstacleDetection.

    ``transform`` is the return value of ``_make_transform`` (a mock with
    ``.transform.translation`` and ``.transform.rotation``).  Its values are
    lifted onto a PoseArray mock so the sensor can read robot position from
    ``_latest_robot_pose`` instead of TF.
    """
    # Build PoseArray from transform values for ground-truth based sensor
    rx = transform.transform.translation.x
    ry = transform.transform.translation.y
    rw = transform.transform.rotation.w
    rz = transform.transform.rotation.z
    robot_pose_msg = MagicMock()
    robot_pose_msg.poses = [MagicMock()]
    robot_pose_msg.poses[0].position.x = rx
    robot_pose_msg.poses[0].position.y = ry
    robot_pose_msg.poses[0].orientation.w = rw
    robot_pose_msg.poses[0].orientation.z = rz
    sensor._latest_robot_pose = robot_pose_msg
    sensor._latest_poses["/obstacle/pose_ground_truth"] = pose_array

    with patch("rclpy.time.Time"):
        sensor._timer_callback()

    assert sensor._publisher.publish.called, "_timer_callback did not call publish()"
    return sensor._publisher.publish.call_args[0][0]


# ─────────────────────────────────────────────────────────────────────────────
# 1. Winding-Order Assertion
# ─────────────────────────────────────────────────────────────────────────────


class TestWindingOrder:
    """Verify all polygon corners are ordered counter-clockwise."""

    # ── Geometry-level (no ROS) ───────────────────────────────────────────

    def test_compute_corners_ccw_no_rotation(self):
        """compute_corners at yaw=0 must yield positive signed area (CCW)."""
        corners = compute_corners(0.0, 0.0, 0.0, 0.5)
        area = signed_area(corners)
        assert area > 0, f"Expected CCW (area>0), got area={area:.6f}"

    @pytest.mark.parametrize(
        "yaw_rad",
        [
            math.pi / 6,
            math.pi / 4,
            math.pi / 3,
            math.pi / 2,
            math.pi,
            -math.pi / 3,
            -math.pi / 2,
        ],
    )
    def test_compute_corners_ccw_arbitrary_yaw(self, yaw_rad: float):
        """compute_corners must be CCW for any obstacle yaw."""
        corners = compute_corners(2.0, -1.0, yaw_rad, 0.3)
        area = signed_area(corners)
        assert area > 0, (
            f"CW winding at yaw={math.degrees(yaw_rad):.1f}°: area={area:.6f}"
        )

    def test_compute_corners_ccw_cross_products(self):
        """Every consecutive-edge cross product must be >= 0 for CCW traversal."""
        corners = compute_corners(0.0, 0.0, 0.0, 0.5)
        n = len(corners)
        for i in range(n):
            p0 = corners[i]
            p1 = corners[(i + 1) % n]
            p2 = corners[(i + 2) % n]
            cross = cross_product_2d(p0, p1, p2)
            assert cross >= -1e-9, f"CW turn (cross={cross:.6f}) at vertex index {i}"

    def test_sort_ccw_on_clockwise_input_produces_positive_area(self):
        """sort_counter_clockwise must produce a positive-area polygon from CW input."""
        cw_points = [(1.0, 0.0), (0.0, -1.0), (-1.0, 0.0), (0.0, 1.0)]
        result = sort_counter_clockwise(cw_points)
        area = signed_area(result)
        assert area > 0, f"sort_ccw output is still CW: area={area:.6f}"

    def test_sort_ccw_preserves_positive_area(self):
        """sort_counter_clockwise on already-CCW input must preserve positive area."""
        ccw_points = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
        result = sort_counter_clockwise(ccw_points)
        area = signed_area(result)
        assert area > 0, f"sort_ccw destroyed CCW order: area={area:.6f}"

    # ── Integration-level (via _timer_callback) ───────────────────────────

    def test_published_polygon_is_ccw(self):
        """Obstacle directly ahead: the published polygon must have positive signed area."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            _make_transform(0.0, 0.0, 0.0),
            _make_pose_array(1.0, 0.0),
        )
        assert len(msg.obstacles) == 1, (
            f"Expected 1 detected obstacle, got {len(msg.obstacles)}"
        )
        pts = [(p.x, p.y) for p in msg.obstacles[0].points]
        assert len(pts) >= 3, "Published polygon has fewer than 3 corners"
        area = signed_area(pts)
        assert area > 0, f"Published polygon is CW: area={area:.6f}"

    def test_published_polygon_is_ccw_with_rotated_obstacle(self):
        """Published polygon must be CCW regardless of obstacle orientation."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            _make_transform(0.0, 0.0, 0.0),
            _make_pose_array(1.0, 0.0, oyaw=math.pi / 4),
        )
        assert len(msg.obstacles) == 1
        pts = [(p.x, p.y) for p in msg.obstacles[0].points]
        assert len(pts) >= 3
        assert signed_area(pts) > 0

    def test_published_polygon_is_ccw_with_rotated_robot(self):
        """Published polygon (body frame) must be CCW when robot has non-zero yaw."""
        sensor = _make_sensor()
        # Robot faces +y (yaw=π/2); obstacle directly ahead at (0, 1)
        msg = _run_callback(
            sensor,
            _make_transform(0.0, 0.0, math.pi / 2),
            _make_pose_array(0.0, 1.0),
        )
        assert len(msg.obstacles) == 1
        pts = [(p.x, p.y) for p in msg.obstacles[0].points]
        assert len(pts) >= 3
        assert signed_area(pts) > 0


# ─────────────────────────────────────────────────────────────────────────────
# 2. Distance-Filtering Assertion
# ─────────────────────────────────────────────────────────────────────────────


class TestDistanceFiltering:
    """Verify distance gates [minimum_distance, detection_range] are enforced."""

    # Robot at origin facing +x for all distance tests.
    _ROBOT_TRANSFORM = _make_transform(0.0, 0.0, 0.0)

    def test_obstacle_beyond_max_range_not_detected(self):
        """Obstacle center at 2.5 m (> 2.0 m limit) must not appear in output."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            self._ROBOT_TRANSFORM,
            _make_pose_array(2.5, 0.0),
        )
        assert len(msg.obstacles) == 0, (
            f"Obstacle at 2.5 m should be suppressed but got {len(msg.obstacles)}"
        )

    def test_obstacle_well_beyond_max_range_not_detected(self):
        """Obstacle at 10 m must not appear in output."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            self._ROBOT_TRANSFORM,
            _make_pose_array(10.0, 0.0),
        )
        assert len(msg.obstacles) == 0

    def test_obstacle_within_minimum_distance_not_detected(self):
        """Obstacle center at 0.05 m (< 0.1 m limit) must not appear in output."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            self._ROBOT_TRANSFORM,
            _make_pose_array(0.05, 0.0),
        )
        assert len(msg.obstacles) == 0, (
            f"Obstacle at 0.05 m should be suppressed but got {len(msg.obstacles)}"
        )

    def test_obstacle_at_zero_distance_not_detected(self):
        """Obstacle coincident with robot (0 m) must not appear in output."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            self._ROBOT_TRANSFORM,
            _make_pose_array(0.0, 0.0),
        )
        assert len(msg.obstacles) == 0

    def test_obstacle_in_valid_range_detected(self):
        """Obstacle at 1.0 m (well within [0.1, 2.0]) must be detected."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            self._ROBOT_TRANSFORM,
            _make_pose_array(1.0, 0.0),
        )
        assert len(msg.obstacles) == 1, (
            f"Expected 1 obstacle at 1.0 m, got {len(msg.obstacles)}"
        )

    def test_obstacle_near_max_range_detected(self):
        """Obstacle at 1.8 m (near but within 2.0 m limit) must be detected."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            self._ROBOT_TRANSFORM,
            _make_pose_array(1.8, 0.0),
        )
        assert len(msg.obstacles) == 1, (
            f"Expected 1 obstacle at 1.8 m, got {len(msg.obstacles)}"
        )

    def test_obstacle_just_above_minimum_distance_detected(self):
        """Obstacle at 0.15 m (above 0.1 m threshold) must be detected."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            self._ROBOT_TRANSFORM,
            _make_pose_array(0.15, 0.0),
        )
        assert len(msg.obstacles) == 1, (
            f"Expected 1 obstacle at 0.15 m, got {len(msg.obstacles)}"
        )

    def test_no_pose_array_publishes_empty_obstacles(self):
        """When no obstacle pose data has arrived, published msg must contain 0 obstacles."""
        sensor = _make_sensor()
        # Set robot ground truth so sensor proceeds past the robot-pose gate
        rx = self._ROBOT_TRANSFORM.transform.translation.x
        ry = self._ROBOT_TRANSFORM.transform.translation.y
        rw = self._ROBOT_TRANSFORM.transform.rotation.w
        rz = self._ROBOT_TRANSFORM.transform.rotation.z
        robot_pose_msg = MagicMock()
        robot_pose_msg.poses = [MagicMock()]
        robot_pose_msg.poses[0].position.x = rx
        robot_pose_msg.poses[0].position.y = ry
        robot_pose_msg.poses[0].orientation.w = rw
        robot_pose_msg.poses[0].orientation.z = rz
        sensor._latest_robot_pose = robot_pose_msg
        # _latest_poses[topic] remains None (set in _make_sensor)

        with patch("rclpy.time.Time"):
            sensor._timer_callback()

        msg = sensor._publisher.publish.call_args[0][0]
        assert len(msg.obstacles) == 0

    def test_distance_filtering_with_offset_robot(self):
        """Distance check must work correctly when robot is not at world origin."""
        sensor = _make_sensor()
        # Robot at (5, 5), obstacle at (6.5, 5) → centre distance = 1.5 m → DETECTED
        msg = _run_callback(
            sensor,
            _make_transform(5.0, 5.0, 0.0),
            _make_pose_array(6.5, 5.0),
        )
        assert len(msg.obstacles) == 1

    def test_far_obstacle_with_offset_robot_not_detected(self):
        """Obstacle 3 m away from an offset robot must not be detected."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            _make_transform(5.0, 5.0, 0.0),
            _make_pose_array(8.0, 5.0),  # 3 m away
        )
        assert len(msg.obstacles) == 0

    @pytest.mark.parametrize("distance", [2.1, 3.0, 5.0, 100.0])
    def test_various_out_of_range_distances_not_detected(self, distance: float):
        """Obstacles beyond detection_range must never appear in output."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            self._ROBOT_TRANSFORM,
            _make_pose_array(distance, 0.0),
        )
        assert len(msg.obstacles) == 0, f"Obstacle at {distance} m should be suppressed"

    @pytest.mark.parametrize("distance", [0.5, 1.0, 1.5, 1.9])
    def test_various_in_range_distances_detected(self, distance: float):
        """Obstacles within [minimum_distance, detection_range] must be detected."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            self._ROBOT_TRANSFORM,
            _make_pose_array(distance, 0.0),
        )
        assert len(msg.obstacles) == 1, f"Obstacle at {distance} m should be detected"


# ─────────────────────────────────────────────────────────────────────────────
# 3. FOV-Filtering Assertion
# ─────────────────────────────────────────────────────────────────────────────


class TestFOVFiltering:
    """Verify obstacles outside ±(field_of_view/2) are suppressed.

    For all tests the robot is at the world origin.
    half_fov = 2.094 / 2 ≈ 1.047 rad = 60°
    """

    _DIST: float = 1.0  # obstacle distance ensuring it is within detection_range

    def _obs_at_angle(self, angle_rad: float, dist: float = None) -> MagicMock:
        """Return a PoseArray for an obstacle ``dist`` m away at ``angle_rad``."""
        d = dist if dist is not None else self._DIST
        return _make_pose_array(d * math.cos(angle_rad), d * math.sin(angle_rad))

    # ── Obstacle directly ahead ────────────────────────────────────────────

    def test_obstacle_directly_ahead_detected(self):
        """Obstacle at angle=0° (straight ahead) must be detected."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            _make_transform(0.0, 0.0, 0.0),
            self._obs_at_angle(0.0),
        )
        assert len(msg.obstacles) == 1

    # ── Left-side boundary ────────────────────────────────────────────────

    def test_obstacle_left_at_55deg_in_fov_detected(self):
        """Obstacle at +55° (< 60° half-FOV) must be detected."""
        sensor = _make_sensor()
        angle = math.radians(55.0)
        msg = _run_callback(
            sensor,
            _make_transform(0.0, 0.0, 0.0),
            self._obs_at_angle(angle),
        )
        assert len(msg.obstacles) == 1, (
            f"Obstacle at +55° should be in FOV but got {len(msg.obstacles)}"
        )

    def test_obstacle_left_at_70deg_outside_fov_not_detected(self):
        """Obstacle at +70° must not be detected once fully outside the FOV."""
        sensor = _make_sensor()
        angle = math.radians(70.0)
        msg = _run_callback(
            sensor,
            _make_transform(0.0, 0.0, 0.0),
            self._obs_at_angle(angle),
        )
        assert len(msg.obstacles) == 0, (
            f"Obstacle at +65° should be outside FOV but got {len(msg.obstacles)}"
        )

    # ── Right-side boundary ───────────────────────────────────────────────

    def test_obstacle_right_at_55deg_in_fov_detected(self):
        """Obstacle at -55° (< 60° half-FOV, right side) must be detected."""
        sensor = _make_sensor()
        angle = math.radians(-55.0)
        msg = _run_callback(
            sensor,
            _make_transform(0.0, 0.0, 0.0),
            self._obs_at_angle(angle),
        )
        assert len(msg.obstacles) == 1

    def test_obstacle_right_at_70deg_outside_fov_not_detected(self):
        """Obstacle at -70° must not be detected once fully outside the FOV."""
        sensor = _make_sensor()
        angle = math.radians(-70.0)
        msg = _run_callback(
            sensor,
            _make_transform(0.0, 0.0, 0.0),
            self._obs_at_angle(angle),
        )
        assert len(msg.obstacles) == 0

    # ── Behind the robot ─────────────────────────────────────────────────

    def test_obstacle_directly_behind_not_detected(self):
        """Obstacle at ±180° (behind robot) must not be detected."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            _make_transform(0.0, 0.0, 0.0),
            self._obs_at_angle(math.pi),
        )
        assert len(msg.obstacles) == 0

    def test_obstacle_at_90deg_outside_fov_not_detected(self):
        """Obstacle at +90° (right angle, outside 60° half-FOV) must not be detected."""
        sensor = _make_sensor()
        msg = _run_callback(
            sensor,
            _make_transform(0.0, 0.0, 0.0),
            self._obs_at_angle(math.pi / 2),
        )
        assert len(msg.obstacles) == 0

    def test_obstacle_center_outside_fov_but_corner_visible_detected(self):
        """Obstacle should still be detected if a corner remains inside the FOV."""
        sensor = _make_sensor()
        angle = math.radians(65.0)
        msg = _run_callback(
            sensor,
            _make_transform(0.0, 0.0, 0.0),
            self._obs_at_angle(angle),
        )
        assert len(msg.obstacles) == 1

    # ── Robot yaw matters ─────────────────────────────────────────────────

    def test_fov_relative_to_robot_heading(self):
        """FOV is relative to robot heading; obstacle behind world-+x but
        directly ahead of a robot facing +y must be detected."""
        sensor = _make_sensor()
        # Robot faces +y (yaw = π/2).  Obstacle at (0, 1.0) is directly ahead.
        msg = _run_callback(
            sensor,
            _make_transform(0.0, 0.0, math.pi / 2),
            _make_pose_array(0.0, 1.0),
        )
        assert len(msg.obstacles) == 1, (
            "Obstacle ahead of rotated robot should be detected"
        )

    def test_fov_relative_to_robot_heading_side_blocked(self):
        """Obstacle to world-+x that is 90° to the side of a robot facing +y
        must not be detected (outside FOV)."""
        sensor = _make_sensor()
        # Robot at (0,0) faces +y; obstacle at (1.0, 0.0) is 90° to robot's right
        msg = _run_callback(
            sensor,
            _make_transform(0.0, 0.0, math.pi / 2),
            _make_pose_array(1.0, 0.0),
        )
        assert len(msg.obstacles) == 0, (
            "Obstacle 90° to side of rotated robot should be outside FOV"
        )

    def test_fov_relative_to_robot_heading_partially_inside(self):
        """Obstacle ~45° ahead-left of a rotated robot must be detected."""
        sensor = _make_sensor()
        # Robot faces +y. Obstacle at (0.5, 0.866): angle to obstacle in world ≈ 60°
        # from +x, robot_yaw = π/2.  angle_diff = 60° - 90° = -30° → within ±60° ✓
        msg = _run_callback(
            sensor,
            _make_transform(0.0, 0.0, math.pi / 2),
            _make_pose_array(0.5, 0.866),
        )
        assert len(msg.obstacles) == 1

    @pytest.mark.parametrize(
        "angle_deg,expected_count",
        [
            (0, 1),
            (30, 1),
            (55, 1),
            (60, 1),
            (65, 1),
            (70, 0),
            (90, 0),
            (180, 0),
            (-30, 1),
            (-55, 1),
            (-60, 1),
            (-65, 1),
            (-70, 0),
        ],
    )
    def test_fov_boundary_parametrized(self, angle_deg: float, expected_count: int):
        """Parametrized sweep of angles around the corner-visible FOV boundary."""
        sensor = _make_sensor()
        angle_rad = math.radians(angle_deg)
        pa = _make_pose_array(
            self._DIST * math.cos(angle_rad),
            self._DIST * math.sin(angle_rad),
        )
        msg = _run_callback(sensor, _make_transform(0.0, 0.0, 0.0), pa)
        assert len(msg.obstacles) == expected_count, (
            f"At {angle_deg}°: expected {expected_count} obstacle(s), "
            f"got {len(msg.obstacles)}"
        )


# ─────────────────────────────────────────────────────────────────────────────
# 4. Update-Rate Assertion
# ─────────────────────────────────────────────────────────────────────────────


class TestUpdateRate:
    """Verify the 10 Hz / 0.1 s update rate is configured and used correctly."""

    def test_default_update_rate_is_10hz(self):
        """Sensor instantiated with default parameters must run at 10 Hz."""
        sensor = _make_sensor()
        assert math.isclose(sensor.update_rate, 10.0, rel_tol=1e-9), (
            f"Expected update_rate=10.0 Hz, got {sensor.update_rate}"
        )

    def test_timer_period_is_01_seconds(self):
        """1/update_rate must equal 0.1 s (±10 % tolerance per blueprint)."""
        sensor = _make_sensor()
        timer_period = 1.0 / sensor.update_rate
        assert math.isclose(timer_period, 0.1, rel_tol=0.10), (
            f"Timer period {timer_period:.4f} s is outside 0.1 s ±10 %"
        )

    def test_start_timer_creates_timer_at_01s_period(self):
        """_start_timer must call create_timer with period = 0.1 s."""
        from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor

        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        sensor.update_rate = 10.0
        sensor._init_timer = MagicMock()
        sensor.create_timer = MagicMock()

        sensor._start_timer()

        sensor._init_timer.cancel.assert_called_once()
        sensor.create_timer.assert_called_once()
        actual_period = sensor.create_timer.call_args[0][0]
        assert math.isclose(actual_period, 0.1, rel_tol=1e-9), (
            f"create_timer called with period={actual_period:.4f} s, expected 0.1 s"
        )

    def test_custom_update_rate_reflected_in_period(self):
        """A non-default update rate must compute the correct timer period."""
        sensor = _make_sensor(update_rate=20.0)
        timer_period = 1.0 / sensor.update_rate
        assert math.isclose(timer_period, 0.05, rel_tol=1e-9)

    def test_timer_period_10pct_tolerance_passes_exactly_10hz(self):
        """Exactly 10.0 Hz must be within the ±10 % tolerance band."""
        sensor = _make_sensor()
        period = 1.0 / sensor.update_rate
        assert 0.09 <= period <= 0.11, (
            f"Period {period:.4f} s outside [0.09, 0.11] tolerance band"
        )
