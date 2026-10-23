import math

from geometry_msgs.msg import Pose, PoseArray, Twist
from rclpy.node import Node
from rectangle_robot.msg import RectangleRobotPath

import rclpy

from ag_labs_py.control_utils import clamp, heading_from_quaternion, wrap_angle

CONTROL_PERIOD_S = 0.01
ANGULAR_PROPORTIONAL_GAIN = 10.0
ANGULAR_DERIVATIVE_GAIN = 3.0
ACCEPTABLE_HEADING_ERROR_RAD = 0.001
MAX_ANGULAR_SPEED_RAD_PER_S = 1.5
LINEAR_PROPORTIONAL_GAIN = 2.0
LINEAR_DERIVATIVE_GAIN = 0.1
GOAL_TOLERANCE_METERS = 0.001
MAX_LINEAR_SPEED_M_PER_S = 1.0


class RectanglePathFollow(Node):
    def __init__(self):
        super().__init__("rectangle_path_follow")

        self._latest_pose: Pose | None = None
        self._path: list[tuple[float, float]] = []
        self._is_in_turning_state = True
        self._target_heading_rad: float | None = None
        self._previous_heading_error_rad: float | None = None
        self._drive_start_position: tuple[float, float] | None = None
        self._target_drive_heading_rad: float | None = None
        self._previous_distance_error: float | None = None

        self.create_subscription(PoseArray, "/rectangle_robot/pose_ground_truth", self._pose_cb, 10)
        self.create_subscription(RectangleRobotPath, "/planning/path", self._path_cb, 10)
        self._vel_pub = self.create_publisher(Twist, "/rectangle_robot/cmd_vel", 10)

        self.create_timer(CONTROL_PERIOD_S, self._control_loop)

    def _pose_cb(self, msg: PoseArray):
        if msg.poses:
            self._latest_pose = next(iter(msg.poses))

    def _path_cb(self, msg: RectangleRobotPath):
        self._path = [(command.distance, command.angle) for command in msg.commands]
        self._reset_command_state()
        self.get_logger().info(f"Received rectangle robot path with {len(self._path)} commands.")

    def _get_robot_pose(self) -> tuple[float, float, float]:
        if self._latest_pose is None:
            raise RuntimeError("No ground truth pose received yet.")
        return (
            self._latest_pose.position.x,
            self._latest_pose.position.y,
            heading_from_quaternion(self._latest_pose.orientation)
        )

    def _control_loop(self):
        if self._latest_pose is None:
            return

        if not self._path:
            return

        cmd = Twist()

        target_distance_meters, target_relative_angle_rad = self._path[0]
        robot_x, robot_y, robot_heading_rad = self._get_robot_pose()

        if self._is_in_turning_state:
            if self._target_heading_rad is None:
                self._target_heading_rad = wrap_angle(robot_heading_rad + target_relative_angle_rad)

            heading_error_rad = wrap_angle(self._target_heading_rad - robot_heading_rad)

            if abs(heading_error_rad) < ACCEPTABLE_HEADING_ERROR_RAD:
                self._is_in_turning_state = False
                self._target_drive_heading_rad = self._target_heading_rad
                self._previous_heading_error_rad = None
                self._vel_pub.publish(cmd)
                return

            if self._previous_heading_error_rad is None:
                self._previous_heading_error_rad = heading_error_rad

            error_diff = wrap_angle(heading_error_rad - self._previous_heading_error_rad)
            heading_error_derivative = error_diff / CONTROL_PERIOD_S
            self._previous_heading_error_rad = heading_error_rad

            angular_speed = (
                ANGULAR_PROPORTIONAL_GAIN * heading_error_rad
            ) + (
                ANGULAR_DERIVATIVE_GAIN * heading_error_derivative
            )
            cmd.angular.z = clamp(
                angular_speed,
                -MAX_ANGULAR_SPEED_RAD_PER_S,
                MAX_ANGULAR_SPEED_RAD_PER_S
            )

            self._vel_pub.publish(cmd)
        else:
            if self._drive_start_position is None:
                self._drive_start_position = (robot_x, robot_y)
                self._previous_distance_error = target_distance_meters

            start_x, start_y = self._drive_start_position
            distance_traveled_meters = math.hypot(robot_x - start_x, robot_y - start_y)
            distance_error_meters = target_distance_meters - distance_traveled_meters

            if distance_error_meters <= GOAL_TOLERANCE_METERS:
                self._path.pop(0)
                self._reset_command_state()
                self._vel_pub.publish(cmd)
                return

            if self._previous_distance_error is None:
                self._previous_distance_error = distance_error_meters

            distance_error_derivative = (
                distance_error_meters - self._previous_distance_error
            ) / CONTROL_PERIOD_S
            self._previous_distance_error = distance_error_meters

            raw_linear_speed = (
                LINEAR_PROPORTIONAL_GAIN * distance_error_meters
            ) + (
                LINEAR_DERIVATIVE_GAIN * distance_error_derivative
            )
            cmd.linear.x = clamp(raw_linear_speed, 0.0, MAX_LINEAR_SPEED_M_PER_S)

            if self._target_drive_heading_rad is not None:
                heading_error_rad = wrap_angle(self._target_drive_heading_rad - robot_heading_rad)
                cmd.angular.z = clamp(
                    ANGULAR_PROPORTIONAL_GAIN * heading_error_rad,
                    -MAX_ANGULAR_SPEED_RAD_PER_S,
                    MAX_ANGULAR_SPEED_RAD_PER_S
                )

            self._vel_pub.publish(cmd)

    def _reset_command_state(self):
        self._is_in_turning_state = True
        self._target_heading_rad = None
        self._previous_heading_error_rad = None
        self._drive_start_position = None
        self._target_drive_heading_rad = None
        self._previous_distance_error = None


def main(args=None):
    rclpy.init(args=args)
    node = RectanglePathFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
