"""Unit tests for mock_obstacle_sensor ground truth robot position."""

import math
from unittest.mock import MagicMock


def test_robot_pose_subscription_topic():
    """Node must subscribe to /racer/pose_ground_truth for robot position."""
    import ag_labs_py.mock_obstacle_sensor as mod

    assert hasattr(mod, 'MockObstacleSensor')
    assert '_latest_robot_pose' in mod.MockObstacleSensor.__init__.__code__.co_names or \
           '_latest_robot_pose' in dir(mod.MockObstacleSensor), \
        'MockObstacleSensor.__init__ must create _latest_robot_pose attribute'
    assert '_on_robot_pose_received' in dir(mod.MockObstacleSensor), \
        'MockObstacleSensor must define _on_robot_pose_received callback'


class TestRobotPoseReceived:
    """Tests for the _on_robot_pose_received callback."""

    def test_stores_latest_pose(self):
        """Callback should store the received PoseArray."""
        from ag_labs_py.mock_obstacle_sensor import MockObstacleSensor
        from geometry_msgs.msg import PoseArray

        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        sensor._on_robot_pose_received = MagicMock(
            wraps=sensor._on_robot_pose_received
        )
        sensor._latest_robot_pose = None

        msg = PoseArray()
        sensor._on_robot_pose_received(msg)

        assert sensor._latest_robot_pose is msg


class TestRobotPoseExtraction:
    """Tests for extracting robot pose from PoseArray."""

    def test_extract_pose_yaw(self):
        """PoseArray poses[0] should give correct x, y, yaw."""
        from ag_labs_py.mock_obstacle_sensor import (
            MockObstacleSensor,
        )
        from geometry_msgs.msg import PoseArray, Pose

        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        sensor._latest_robot_pose = PoseArray()
        pose = Pose()
        pose.position.x = 1.5
        pose.position.y = -2.3
        pose.orientation.w = math.cos(0.5 / 2)
        pose.orientation.z = math.sin(0.5 / 2)
        sensor._latest_robot_pose.poses = [pose]

        robot_x = sensor._latest_robot_pose.poses[0].position.x
        robot_y = sensor._latest_robot_pose.poses[0].position.y
        robot_yaw = sensor._quaternion_to_yaw(
            sensor._latest_robot_pose.poses[0].orientation
        )

        assert math.isclose(robot_x, 1.5)
        assert math.isclose(robot_y, -2.3)
        assert math.isclose(robot_yaw, 0.5, abs_tol=1e-9)

    def test_empty_pose_array(self):
        """Empty PoseArray should not crash."""
        from ag_labs_py.mock_obstacle_sensor import (
            MockObstacleSensor,
        )
        from geometry_msgs.msg import PoseArray

        sensor = MockObstacleSensor.__new__(MockObstacleSensor)
        sensor._latest_robot_pose = PoseArray()
        sensor._latest_robot_pose.poses = []

        assert len(sensor._latest_robot_pose.poses) == 0
