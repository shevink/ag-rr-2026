import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from ag_labs_py.control_utils import clamp

from .path_math import (
    MAX_STEERING_ANGLE_DEGREES,
    MAX_STEERING_ANGLE_RADIANS,
    MAX_ALLOWED_CURVATURE,
    SPLINE_SAMPLE_SPACING_METERS,
    VEHICLE_LENGTH_METERS,
    VEHICLE_WIDTH_METERS,
    WHEELBASE_METERS,
    ActivePath,
    PathPoint,
    calculate_path_sample_distances,
    distance_between,
    drop_duplicate_points,
    maximum_required_steering,
    normalize_angle,
    smooth_path_with_natural_cubic_splines,
    yaw_from_quaternion,
)


CONTROL_PERIOD_SECONDS = 0.02
COMMAND_LINEAR_SPEED = 0.26
CURVE_LINEAR_SPEED = 0.18
MINIMUM_LINEAR_SPEED = 0.04
SLOWDOWN_DISTANCE_METERS = 0.25
LOOKAHEAD_DISTANCE_METERS = 0.45
GOAL_TOLERANCE_METERS = 0.001
WAYPOINT_ADVANCE_DISTANCE_METERS = 0.025
CURVE_SPEED_CURVATURE_THRESHOLD = 0.35
MINIMUM_CLOSED_PATH_COMPLETION_RATIO = 0.96
CLOSED_RETURN_TOLERANCE_METERS = 0.04
CLOSED_RETURN_DISTANCE_INCREASE_TOLERANCE_METERS = 0.02

BASE_FRAME_IDS = {"base_footprint", "racer/base_footprint"}
PATH_TOPIC = "planning/path"
SMOOTHED_PATH_TOPIC = "planning/smoothed_path"
ODOMETRY_TOPIC = "/racer/pose_ground_truth"
COMMAND_TOPIC = "/racer/cmd_vel"


class KinematicPathFollower(Node):
    def __init__(self):
        super().__init__("kinematic_path_follower")

        self._latest_pose: Pose | None = None
        self._active_path: ActivePath | None = None

        self.create_subscription(PoseArray, ODOMETRY_TOPIC, self._pose_callback, 10)
        self.create_subscription(Path, PATH_TOPIC, self._path_callback, 10)
        self._velocity_publisher = self.create_publisher(Twist, COMMAND_TOPIC, 10)
        smoothed_path_qos = QoSProfile(depth=1)
        smoothed_path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._smoothed_path_publisher = self.create_publisher(
            Path, SMOOTHED_PATH_TOPIC, smoothed_path_qos
        )
        self.create_timer(CONTROL_PERIOD_SECONDS, self._control_loop)

        self.get_logger().info(
            "Kinematic path follower ready: "
            f"{VEHICLE_LENGTH_METERS:.2f}x{VEHICLE_WIDTH_METERS:.2f} m footprint, "
            f"{MAX_STEERING_ANGLE_DEGREES:.1f} deg steering limit"
        )

    def _pose_callback(self, message: PoseArray):
        if message.poses:
            self._latest_pose = next(iter(message.poses))

    def _path_callback(self, message: Path):
        self._active_path = None

        if len(message.poses) < 2:
            self.get_logger().warn("Rejected path: at least two poses are required")
            self._publish_smoothed_path([], message.header.frame_id)
            self._publish_stop()
            return

        try:
            input_points = self._path_message_points(message)
        except RuntimeError as error:
            self.get_logger().warn(f"Rejected path: {error}")
            self._publish_smoothed_path([], message.header.frame_id)
            self._publish_stop()
            return

        input_points = drop_duplicate_points(input_points)
        if len(input_points) < 2:
            self.get_logger().warn("Rejected path: unique path length is too short")
            self._publish_smoothed_path([], message.header.frame_id)
            self._publish_stop()
            return

        try:
            smoothed_display_path = smooth_path_with_natural_cubic_splines(input_points)
        except ValueError as error:
            self.get_logger().warn(f"Rejected path: spline generation failed: {error}")
            self._publish_smoothed_path([], message.header.frame_id)
            self._publish_stop()
            return

        steering_angle, curvature = maximum_required_steering(smoothed_display_path)
        if steering_angle > MAX_STEERING_ANGLE_RADIANS:
            self.get_logger().warn(
                "Rejected path: required steering angle "
                f"{math.degrees(steering_angle):.1f} deg exceeds "
                f"{MAX_STEERING_ANGLE_DEGREES:.1f} deg "
                f"(max curvature {curvature:.3f} 1/m, "
                f"vehicle footprint {VEHICLE_LENGTH_METERS:.2f}x"
                f"{VEHICLE_WIDTH_METERS:.2f} m)"
            )
            self._publish_smoothed_path([], message.header.frame_id)
            self._publish_stop()
            return

        try:
            smoothed_path = self._control_path_points(
                smoothed_display_path, message.header.frame_id
            )
        except RuntimeError as error:
            self.get_logger().warn(f"Rejected path: {error}")
            self._publish_smoothed_path([], message.header.frame_id)
            self._publish_stop()
            return
        self._active_path = ActivePath(
            points=smoothed_path,
            sample_distances=calculate_path_sample_distances(smoothed_path),
            is_closed=self._path_returns_to_start(smoothed_path),
        )
        self._publish_smoothed_path(
            smoothed_display_path, message.header.frame_id or "odom"
        )

        self.get_logger().info(
            f"Accepted path: {len(message.poses)} input poses, "
            f"{len(smoothed_path)} spline samples, "
            f"length={self._active_path.length:.2f} m, "
            f"max steering={math.degrees(steering_angle):.1f} deg, "
            f"closed={self._active_path.is_closed}"
        )

    def _path_message_points(self, message: Path) -> list[PathPoint]:
        message_points = [
            PathPoint(pose_stamped.pose.position.x, pose_stamped.pose.position.y)
            for pose_stamped in message.poses
        ]

        if message.header.frame_id in BASE_FRAME_IDS:
            return self._ensure_path_starts_at_origin(message_points)

        return message_points

    def _control_path_points(
        self, path_points: list[PathPoint], frame_id: str
    ) -> list[PathPoint]:
        if frame_id not in BASE_FRAME_IDS:
            return path_points

        current_pose = self._current_pose()
        if current_pose is None:
            raise RuntimeError("no racer pose is available for relative path transform")

        current_x, current_y, current_heading = current_pose
        cos_heading = math.cos(current_heading)
        sin_heading = math.sin(current_heading)

        transformed_points: list[PathPoint] = []
        for path_point in path_points:
            odom_x = (
                current_x
                + cos_heading * path_point.position_forward
                - sin_heading * path_point.position_left
            )
            odom_y = (
                current_y
                + sin_heading * path_point.position_forward
                + cos_heading * path_point.position_left
            )
            transformed_points.append(
                PathPoint(
                    odom_x,
                    odom_y,
                    normalize_angle(current_heading + path_point.heading_radians),
                    path_point.curvature,
                )
            )
        return transformed_points

    def _ensure_path_starts_at_origin(
        self, path_points: list[PathPoint]
    ) -> list[PathPoint]:
        if not path_points:
            return path_points

        origin = PathPoint(0.0, 0.0)
        if distance_between(origin, path_points[0]) <= SPLINE_SAMPLE_SPACING_METERS:
            return path_points
        return [origin] + path_points

    def _current_pose(self) -> tuple[float, float, float] | None:
        if self._latest_pose is None:
            return None
        return (
            self._latest_pose.position.x,
            self._latest_pose.position.y,
            yaw_from_quaternion(self._latest_pose.orientation),
        )

    def _control_loop(self):
        current_pose = self._current_pose()
        if current_pose is None or self._active_path is None:
            # No active path: stay silent so /cmd_vel/path goes stale and the
            # twist_mux hands control down to wasd / the zero-velocity floor.
            # Continuously publishing a stop here keeps this (highest-priority)
            # topic permanently active and masks manual teleop. A completed or
            # rejected path already emits a single stop via _publish_stop().
            return

        active_path = self._active_path
        current_position = PathPoint(*current_pose)
        self._update_target_index(active_path, current_position)

        nearest_path_point = active_path.points[active_path.target_index]
        target_point = self._select_lookahead_point(active_path)
        distance_to_goal = distance_between(current_position, active_path.points[-1])
        distance_to_start = distance_between(current_position, active_path.points[0])

        if self._path_is_complete(active_path, distance_to_start, distance_to_goal):
            self.get_logger().info(
                "Completed path: "
                f"distance_to_start={distance_to_start:.2f} m, "
                f"distance_to_goal={distance_to_goal:.2f} m"
            )
            self._active_path = None
            self._publish_stop()
            return

        steering_angle = self._pure_pursuit_steering_angle(
            current_position, target_point
        )
        steering_angle = clamp(
            steering_angle, -MAX_STEERING_ANGLE_RADIANS, MAX_STEERING_ANGLE_RADIANS
        )

        speed = self._speed_for(
            active_path, nearest_path_point, distance_to_start, distance_to_goal
        )
        command = Twist()
        command.linear.x = speed
        command.angular.z = speed * math.tan(steering_angle) / WHEELBASE_METERS
        self._velocity_publisher.publish(command)

    def _update_target_index(
        self, active_path: ActivePath, current_position: PathPoint
    ):
        if active_path.is_closed:
            nearest_index = self._nearest_path_index(active_path, current_position, 0)
            previous_target_index = active_path.target_index
            final_section_index = int(0.85 * (len(active_path.points) - 1))
            start_section_index = int(0.15 * (len(active_path.points) - 1))
            if (
                active_path.maximum_progress_ratio
                >= MINIMUM_CLOSED_PATH_COMPLETION_RATIO
                and previous_target_index >= final_section_index
                and nearest_index <= start_section_index
            ):
                active_path.completed_closed_lap = True

            active_path.target_index = nearest_index
            active_path.maximum_progress_ratio = max(
                active_path.maximum_progress_ratio,
                active_path.distance_at(nearest_index) / max(active_path.length, 1e-6),
            )
            return

        nearest_index = self._nearest_path_index(
            active_path, current_position, active_path.target_index
        )
        active_path.target_index = max(active_path.target_index, nearest_index)

        while active_path.target_index < len(active_path.points) - 1:
            target_point = active_path.points[active_path.target_index]
            if (
                distance_between(current_position, target_point)
                >= WAYPOINT_ADVANCE_DISTANCE_METERS
            ):
                break
            active_path.target_index += 1

    def _nearest_path_index(
        self, active_path: ActivePath, current_position: PathPoint, start_index: int
    ) -> int:
        if active_path.is_closed:
            search_range = range(len(active_path.points))
        else:
            search_range = range(start_index, len(active_path.points))

        nearest_index = active_path.target_index
        nearest_distance = float("inf")
        for candidate_index in search_range:
            candidate_distance = distance_between(
                current_position, active_path.points[candidate_index]
            )
            if candidate_distance < nearest_distance:
                nearest_index = candidate_index
                nearest_distance = candidate_distance
        return nearest_index

    def _select_lookahead_point(self, active_path: ActivePath) -> PathPoint:
        target_distance = (
            active_path.distance_at(active_path.target_index)
            + LOOKAHEAD_DISTANCE_METERS
        )
        if active_path.is_closed and active_path.length > 0.0:
            target_distance = math.fmod(target_distance, active_path.length)
        else:
            target_distance = min(target_distance, active_path.length)

        return active_path.points[active_path.index_at_distance(target_distance)]

    def _pure_pursuit_steering_angle(
        self, current_position: PathPoint, target_point: PathPoint
    ) -> float:
        heading_to_target = math.atan2(
            target_point.position_left - current_position.position_left,
            target_point.position_forward - current_position.position_forward,
        )
        heading_error = normalize_angle(
            heading_to_target - current_position.heading_radians
        )
        lookahead_distance = max(distance_between(current_position, target_point), 1e-6)
        curvature_command = clamp(
            2.0 * math.sin(heading_error) / lookahead_distance,
            -MAX_ALLOWED_CURVATURE,
            MAX_ALLOWED_CURVATURE,
        )
        return math.atan(WHEELBASE_METERS * curvature_command)

    def _speed_for(
        self,
        active_path: ActivePath,
        nearest_path_point: PathPoint,
        distance_to_start: float,
        distance_to_goal: float,
    ) -> float:
        path_speed_limit = COMMAND_LINEAR_SPEED
        if abs(nearest_path_point.curvature) >= CURVE_SPEED_CURVATURE_THRESHOLD:
            path_speed_limit = CURVE_LINEAR_SPEED

        slowdown_distance = active_path.remaining_distance()
        if not active_path.is_closed:
            slowdown_distance = min(slowdown_distance, distance_to_goal)
            if active_path.target_index >= len(active_path.points) - 2:
                slowdown_distance = distance_to_goal
        elif active_path.completed_closed_lap:
            slowdown_distance = distance_to_start

        speed = min(
            path_speed_limit,
            path_speed_limit * slowdown_distance / SLOWDOWN_DISTANCE_METERS,
        )
        is_near_open_path_goal = (
            not active_path.is_closed
            and active_path.target_index >= len(active_path.points) - 2
        )
        is_near_closed_path_goal = (
            active_path.is_closed and active_path.completed_closed_lap
        )
        if speed > 0.0 and not is_near_open_path_goal and not is_near_closed_path_goal:
            speed = max(MINIMUM_LINEAR_SPEED, speed)
        return speed

    def _path_is_complete(
        self,
        active_path: ActivePath,
        distance_to_start: float,
        distance_to_goal: float,
    ) -> bool:
        if active_path.is_closed:
            if active_path.completed_closed_lap:
                active_path.closest_closed_return_distance = min(
                    active_path.closest_closed_return_distance,
                    distance_to_start,
                )

            return (
                active_path.completed_closed_lap
                and self._closed_path_return_is_complete(
                    active_path, distance_to_start
                )
            )

        is_at_final_samples = active_path.target_index >= len(active_path.points) - 2
        return is_at_final_samples and (
            distance_to_goal <= GOAL_TOLERANCE_METERS
            or self._has_passed_open_path_goal(active_path)
        )

    def _closed_path_return_is_complete(
        self, active_path: ActivePath, distance_to_start: float
    ) -> bool:
        if distance_to_start <= CLOSED_RETURN_TOLERANCE_METERS:
            return True

        return (
            active_path.closest_closed_return_distance < float("inf")
            and distance_to_start
            > active_path.closest_closed_return_distance
            + CLOSED_RETURN_DISTANCE_INCREASE_TOLERANCE_METERS
        )

    def _has_passed_open_path_goal(self, active_path: ActivePath) -> bool:
        current_pose = self._current_pose()
        if current_pose is None or len(active_path.points) < 2:
            return False

        current_position = PathPoint(*current_pose)
        previous_goal_point = active_path.points[-2]
        goal_point = active_path.points[-1]
        goal_segment_forward = (
            goal_point.position_forward - previous_goal_point.position_forward
        )
        goal_segment_left = goal_point.position_left - previous_goal_point.position_left
        current_past_goal_forward = (
            current_position.position_forward - goal_point.position_forward
        )
        current_past_goal_left = (
            current_position.position_left - goal_point.position_left
        )
        return (
            goal_segment_forward * current_past_goal_forward
            + goal_segment_left * current_past_goal_left
        ) > 0.0

    def _path_returns_to_start(self, path_points: list[PathPoint]) -> bool:
        if len(path_points) < 2:
            return False
        return distance_between(path_points[0], path_points[-1]) <= LOOKAHEAD_DISTANCE_METERS

    def _publish_smoothed_path(self, path_points: list[PathPoint], frame_id: str):
        path_message = Path()
        path_message.header.frame_id = frame_id or "odom"
        path_message.header.stamp = self.get_clock().now().to_msg()

        for path_point in path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_message.header
            pose_stamped.pose.position.x = path_point.position_forward
            pose_stamped.pose.position.y = path_point.position_left
            pose_stamped.pose.orientation.z = math.sin(path_point.heading_radians / 2.0)
            pose_stamped.pose.orientation.w = math.cos(path_point.heading_radians / 2.0)
            path_message.poses.append(pose_stamped)

        self._smoothed_path_publisher.publish(path_message)

    def _publish_stop(self):
        self._velocity_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = KinematicPathFollower()
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
