import math

from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Path
from rclpy.node import Node

import rclpy

from ag_labs_py.control_utils import clamp, heading_from_quaternion, wrap_angle


# TODO: If this node ever makes it into the team's repos, all of this should be ROS parameters.
# Unit convention: _S = seconds, _M = meters, _RAD = radians, _PER_ = division (e.g. _M_PER_S = m/s)
CONTROL_PERIOD_S = 0.01
ANGULAR_P_GAIN = 10.0
ANGULAR_D_GAIN = 3.0
ACCEPTABLE_HEADING_ERROR_RAD = 0.02
GOAL_TOLERANCE_METERS = 0.01
LINEAR_P_GAIN = 2.0
LINEAR_D_GAIN = 0.1
MAX_LINEAR_SPEED_M_PER_S = 1.0


class NaivePathFollow(Node):
    def __init__(self):
        super().__init__("naive_path_follow")

        self._path: list[tuple[float, float]] = []
        self._current_waypoint_index = 0

        self._latest_pose: Pose | None = None

        self.create_subscription(PoseArray, "/disc_robot/pose_ground_truth", self._pose_cb, 10)
        self.create_subscription(Path, "/planning/path", self._path_cb, 10)
        self._vel_pub = self.create_publisher(Twist, "/disc_robot/cmd_vel", 10)
        self.create_timer(CONTROL_PERIOD_S, self._control_loop)

        self._is_in_turning_state: bool = True
        self._previous_heading_error_rad: float | None = None
        self._previous_distance: float | None = None


    def _pose_cb(self, msg: PoseArray):
        if msg.poses:
            # Somehow msg.poses is a set, and therefore does not support the [] operator.
            # Pylance complains when it is used, so this workaround uses an iterator to 
            # access the element. If I knew any better, I would have explained it better :).
            self._latest_pose = next(iter(msg.poses))

    def _get_robot_pose(self) -> tuple[float, float, float]:
        if self._latest_pose is None:
            raise RuntimeError("No ground truth pose received yet.")
        return (
            self._latest_pose.position.x,
            self._latest_pose.position.y,
            heading_from_quaternion(self._latest_pose.orientation)
        )

    def _path_cb(self, msg: Path):
        # TODO: Only paths in the robot frame or the global frame are handled correctly. The else branch
        # implicitly assumes any frame != the robots frame to be the global frame. A proper implementation
        # would use the TF tree to transform arbitrary frames, but since nobody is likely to publish a path
        # relative to, say, a wheel, I really can't be bothered to fix this at the moment.
        if msg.header.frame_id == "disc_robot/base_footprint":
            try:
                robot_position_x_in_odom, robot_position_y_in_odom, robot_heading_in_odom = self._get_robot_pose()
            except RuntimeError as e:
                self.get_logger().warn(f"Could not transform path: {e}")
                return
            cos_heading = math.cos(robot_heading_in_odom)
            sin_heading = math.sin(robot_heading_in_odom)
            self._path = []
            for p in msg.poses:
                pathpoint_position_x_in_odom = robot_position_x_in_odom + cos_heading * p.pose.position.x - sin_heading * p.pose.position.y
                pathpoint_position_y_in_odom = robot_position_y_in_odom + sin_heading * p.pose.position.x + cos_heading * p.pose.position.y
                self._path.append((pathpoint_position_x_in_odom, pathpoint_position_y_in_odom))
        else:
            self._path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

        self._current_waypoint_index = 0
        self._is_in_turning_state = True
        self._previous_heading_error_rad = None
        self._previous_distance = None

    def _control_loop(self):
        if self._latest_pose is None:
            return

        if not self._path or self._current_waypoint_index >= len(self._path):
            return

        cmd = Twist()    
        
        current_waypoint_x, current_waypoint_y = self._path[self._current_waypoint_index]

        # these angles lie on the unit circle as follows: (1,0) = 0, (0,1) = pi/2, (-1,0) = pi and -pi, (0,-1) = -pi/2
        robot_x, robot_y, robot_heading_angle_rad = self._get_robot_pose()
        angle_towards_next_waypoint_rad: float = math.atan2(current_waypoint_y - robot_y, current_waypoint_x - robot_x)

        # raw_heading_error is the difference between the current robot's heading relative to the X-axis and
        # the angle of the next waypoint relative to the X-axis.
        raw_heading_error: float = angle_towards_next_waypoint_rad - robot_heading_angle_rad
        
        # When simply subtracting the angles one must keep in mind that -pi - pi is not -2pi, but 0, because
        # all of that angle stuff happens on a circle and therefore wraps around. This corrects this.
        # heading_error_rad is the wrapped angle.
        heading_error_rad: float = wrap_angle(raw_heading_error)

        # Euclidean distance
        distance_robot_to_target_point_meters: float = math.hypot(current_waypoint_x - robot_x, current_waypoint_y - robot_y)

        if self._previous_heading_error_rad is None:
            self._previous_heading_error_rad = heading_error_rad

        error_diff = wrap_angle(heading_error_rad - self._previous_heading_error_rad)
        heading_error_derivative = error_diff / CONTROL_PERIOD_S

        self._previous_heading_error_rad = heading_error_rad

        if self._previous_distance is None:
            self._previous_distance = distance_robot_to_target_point_meters

        distance_diff = distance_robot_to_target_point_meters - self._previous_distance
        distance_derivative = distance_diff / CONTROL_PERIOD_S
        self._previous_distance = distance_robot_to_target_point_meters

        if self._is_in_turning_state:

            # The main idea is simple, set z = 1/-1 (to make the robot turn) and periodically check back in (timer)
            # if the heading error is (almost) 0. However, due to physical inertia the robot cannot just hard stop turning.
            # To account for that, we use a PD controller (Proportional-Derivative) to command the turning speed.
            # The derivative term acts as a damper, countersteering when the robot spins too fast to prevent oversteer.
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = (ANGULAR_P_GAIN * heading_error_rad) + (ANGULAR_D_GAIN * heading_error_derivative)

            self._is_in_turning_state = abs(heading_error_rad) >= ACCEPTABLE_HEADING_ERROR_RAD
        elif distance_robot_to_target_point_meters >= GOAL_TOLERANCE_METERS:
            # PD controller for linear velocity to prevent overshooting the waypoint
            raw_linear_speed = (LINEAR_P_GAIN * distance_robot_to_target_point_meters) - (LINEAR_D_GAIN * distance_derivative)
            cmd.linear.x = clamp(raw_linear_speed, 0.0, MAX_LINEAR_SPEED_M_PER_S)
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = (ANGULAR_P_GAIN * heading_error_rad) + (ANGULAR_D_GAIN * heading_error_derivative)
        else:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = 0.0

            self._is_in_turning_state = True
            self._previous_heading_error_rad = None
            self._previous_distance = None
            self._current_waypoint_index += 1
        
        self._vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NaivePathFollow()
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
