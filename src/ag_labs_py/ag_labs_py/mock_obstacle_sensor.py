import math
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Point, Point32, Polygon, PoseArray
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from ag_labs_interfaces.msg import ObstacleDetection
from ag_labs_py.obstacle_geometry import (
    compute_corners,
    sort_counter_clockwise,
    world_to_body,
)

DEFAULT_FIELD_OF_VIEW = math.pi / 2.0
DEFAULT_CONFIG_PATH = str(Path(__file__).parent / 'config.yaml')

# RGBA helper for debug markers
_C_GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
_C_RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
_C_YELLOW = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5)
_C_CYAN = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.3)
_C_WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8)
_C_ORANGE = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)
_C_MAGENTA = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)
_C_GREY = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.3)

_NS_FOV = 'fov'
_NS_RANGE = 'detection_range'
_NS_OBSTACLE = 'obstacle'
_NS_CENTER = 'obstacle_center'
_NS_VERTICES = 'obstacle_vertices'
_NS_FILTERED = 'filtered_obstacle'
_NS_ROBOT = 'robot_frame'


def _parameter_as_list(value) -> list:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    return [value]


def _obstacle_topics_from_config(config_path: str) -> list[str]:
    with open(config_path, 'r') as config_file:
        config = yaml.safe_load(config_file) or {}

    obstacle_names = [
        *config.get('dynamic', {}).keys(),
        *config.get('static', {}).keys(),
    ]
    return [f'/{name}/pose_ground_truth' for name in obstacle_names]


def _make_marker(
    ns: str,
    marker_id: int,
    marker_type: int,
    frame_id: str,
    stamp: rclpy.time.Time,
    lifetime: float,
    color: ColorRGBA,
    scale_x: float = 0.1,
    scale_y: float = 0.1,
    scale_z: float = 0.1,
) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = marker_type
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = scale_x
    m.scale.y = scale_y
    m.scale.z = scale_z
    m.color = color
    m.lifetime = rclpy.duration.Duration(seconds=lifetime).to_msg()
    return m


class MockObstacleSensor(Node):
    def __init__(self) -> None:
        super().__init__('mock_obstacle_sensor')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', 10.0),
                ('detection_range', 2.0),
                ('minimum_distance', 0.1),
                ('field_of_view', DEFAULT_FIELD_OF_VIEW),
                ('source_frame', 'racer/base_footprint'),
                ('auto_configure_obstacles', True),
                ('config_path', DEFAULT_CONFIG_PATH),
                ('obstacle_topics', ['/obstacle/pose_ground_truth']),
                ('obstacle_sizes', [0.2]),
                ('enable_debug', True),
            ],
        )

        self.update_rate = self.get_parameter('update_rate').value
        self.detection_range = self.get_parameter('detection_range').value
        self.minimum_distance = self.get_parameter('minimum_distance').value
        self.field_of_view = self.get_parameter('field_of_view').value
        self.source_frame = self.get_parameter('source_frame').value
        self.auto_configure_obstacles = self.get_parameter(
            'auto_configure_obstacles'
        ).value
        self.config_path = self.get_parameter('config_path').value
        self.enable_debug = self.get_parameter('enable_debug').value
        configured_topics = _parameter_as_list(
            self.get_parameter('obstacle_topics').value
        )
        self.obstacle_topics = self._resolve_obstacle_topics(
            configured_topics
        )
        self.obstacle_sizes = _parameter_as_list(
            self.get_parameter('obstacle_sizes').value
        )

        self._latest_robot_pose: PoseArray | None = None
        self._robot_pose_sub = self.create_subscription(
            PoseArray,
            '/racer/pose_ground_truth',
            self._on_robot_pose_received,
            10,
        )

        self._latest_poses: dict[str, PoseArray | None] = {}
        self._obstacle_subs: list = []
        for topic in self.obstacle_topics:
            self._latest_poses[topic] = None
            self._obstacle_subs.append(
                self.create_subscription(
                    PoseArray,
                    topic,
                    lambda msg, t=topic: self._on_pose_received(msg, t),
                    10,
                )
            )

        self._publisher = self.create_publisher(
            ObstacleDetection,
            '/perception/obstacle',
            10,
        )

        self._debug_pub = self.create_publisher(
            MarkerArray,
            '/perception/debug/obstacle_sensor',
            10,
        )

        self._init_timer = self.create_timer(0.5, self._start_timer)

    def _resolve_obstacle_topics(
        self,
        configured_topics: list[str],
    ) -> list[str]:
        if self.auto_configure_obstacles:
            try:
                auto_topics = _obstacle_topics_from_config(self.config_path)
            except (OSError, yaml.YAMLError) as e:
                self.get_logger().warning(
                    f'Could not load obstacle topics from {self.config_path}: '
                    f'{e}. Falling back to obstacle_topics parameter.'
                )
            else:
                if auto_topics:
                    self.get_logger().info(
                        'Configured obstacle topics from config: '
                        f'{auto_topics}'
                    )
                    return auto_topics

        return configured_topics

    def _start_timer(self) -> None:
        self._init_timer.cancel()
        timer_period = 1.0 / self.update_rate
        self.create_timer(timer_period, self._timer_callback)

    def _on_pose_received(self, msg: PoseArray, topic: str) -> None:
        self._latest_poses[topic] = msg

    def _on_robot_pose_received(self, msg: PoseArray) -> None:
        self._latest_robot_pose = msg

    def _timer_callback(self) -> None:
        if self._latest_robot_pose is None or not self._latest_robot_pose.poses:
            self.get_logger().warning(
                'No robot ground truth pose received yet'
            )
            return

        robot_pose = self._latest_robot_pose.poses[0]
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        robot_yaw = self._quaternion_to_yaw(robot_pose.orientation)

        half_fov = self.field_of_view / 2.0
        obstacles: list[Polygon] = []

        debug_markers: list[Marker] = []
        marker_id = [0]

        def next_id() -> int:
            i = marker_id[0]
            marker_id[0] += 1
            return i

        stamp = self.get_clock().now()
        stamp_msg = stamp.to_msg()

        if getattr(self, 'enable_debug', False):
            clear = Marker()
            clear.action = Marker.DELETEALL
            debug_markers.append(clear)

            debug_markers.extend(self._make_fov_markers(
                half_fov, stamp_msg, next_id,
            ))
            debug_markers.extend(self._make_range_markers(
                stamp_msg, next_id
            ))
            debug_markers.append(
                self._make_robot_frame_marker(stamp_msg, next_id)
            )

        for i, topic in enumerate(self.obstacle_topics):
            size = (
                self.obstacle_sizes[i]
                if i < len(self.obstacle_sizes)
                else self.obstacle_sizes[-1]
            )
            half_size = size / 2.0
            pose_array = self._latest_poses.get(topic)
            if pose_array is None or not pose_array.poses:
                continue

            for pose in pose_array.poses:
                ox = pose.position.x
                oy = pose.position.y
                oyaw = self._quaternion_to_yaw(pose.orientation)

                dx = ox - robot_x
                dy = oy - robot_y
                center_distance = math.hypot(dx, dy)

                filter_reason = self._check_filters(
                    ox, oy, robot_x, robot_y, robot_yaw,
                    center_distance, half_fov,
                )

                world_corners = compute_corners(ox, oy, oyaw, half_size)
                body_corners_all = [
                    world_to_body(c, robot_x, robot_y, robot_yaw)
                    for c in world_corners
                ]
                body_center = world_to_body(
                    (ox, oy), robot_x, robot_y, robot_yaw
                )

                if filter_reason is None:
                    visible_corners = []
                    for bc in body_corners_all:
                        vx, vy = bc
                        vertex_distance = math.hypot(vx, vy)
                        vertex_angle = math.atan2(vy, vx)
                        if (
                            vertex_distance <= self.detection_range
                            and vertex_distance >= self.minimum_distance
                            and abs(vertex_angle) <= half_fov
                        ):
                            visible_corners.append(bc)

                    if not visible_corners:
                        filter_reason = 'out_of_fov'
                    else:
                        sorted_corners = sort_counter_clockwise(visible_corners)
                        polygon = Polygon()
                        for vx, vy in sorted_corners:
                            point = Point32()
                            point.x = vx
                            point.y = vy
                            point.z = 0.0
                            polygon.points.append(point)
                        obstacles.append(polygon)

                if getattr(self, 'enable_debug', False):
                    debug_markers.extend(self._make_obstacle_debug_markers(
                        body_center, body_corners_all,
                        center_distance, filter_reason,
                        topic, stamp_msg, next_id,
                    ))

        msg = ObstacleDetection()
        msg.header = Header()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self.source_frame
        msg.obstacles = obstacles
        self._publisher.publish(msg)

        if getattr(self, 'enable_debug', False) and debug_markers:
            self._debug_pub.publish(MarkerArray(markers=debug_markers))

    def _check_filters(
        self,
        ox: float, oy: float,
        robot_x: float, robot_y: float, robot_yaw: float,
        center_distance: float,
        half_fov: float,
    ) -> str | None:
        if center_distance < self.minimum_distance:
            return 'too_close'
        if center_distance > self.detection_range:
            return 'too_far'
        return None

    def _make_fov_markers(
        self,
        half_fov: float,
        stamp: rclpy.time.Time,
        next_id,
    ) -> list[Marker]:
        markers = []
        segs = 32
        r = self.detection_range

        # Left and right boundary rays in body frame
        body_pts_left = [(0.0, 0.0)]
        body_pts_right = [(0.0, 0.0)]
        for t in [0.0, r]:
            body_pts_left.append((
                t * math.cos(half_fov),
                t * math.sin(half_fov),
            ))
            body_pts_right.append((
                t * math.cos(-half_fov),
                t * math.sin(-half_fov),
            ))
        for label, body_pts in [('fov_left', body_pts_left), ('fov_right', body_pts_right)]:
            m = _make_marker(
                _NS_FOV, next_id(), Marker.LINE_STRIP,
                self.source_frame, stamp, 0.15, _C_CYAN,
                scale_x=0.02,
            )
            for bx, by in body_pts:
                p = Point()
                p.x, p.y, p.z = bx, by, 0.0
                m.points.append(p)
            markers.append(m)

        # FOV arc at detection_range in body frame
        arc = _make_marker(
            _NS_FOV, next_id(), Marker.LINE_STRIP,
            self.source_frame, stamp, 0.15, _C_CYAN,
            scale_x=0.02,
        )
        for i in range(segs + 1):
            a = -half_fov + (2.0 * half_fov) * i / segs
            bx = r * math.cos(a)
            by = r * math.sin(a)
            p = Point()
            p.x, p.y, p.z = bx, by, 0.0
            arc.points.append(p)
        markers.append(arc)

        # Filled FOV cone (triangles) in body frame
        cone = _make_marker(
            _NS_FOV, next_id(), Marker.TRIANGLE_LIST,
            self.source_frame, stamp, 0.15, _C_CYAN,
        )
        cone.scale.x = cone.scale.y = cone.scale.z = 1.0
        for i in range(segs):
            a1 = -half_fov + (2.0 * half_fov) * i / segs
            a2 = -half_fov + (2.0 * half_fov) * (i + 1) / segs
            for bx, by in [(0.0, 0.0),
                           (r * math.cos(a1), r * math.sin(a1)),
                           (r * math.cos(a2), r * math.sin(a2))]:
                pt = Point()
                pt.x, pt.y, pt.z = bx, by, 0.0
                cone.points.append(pt)
        markers.append(cone)

        return markers

    def _make_range_markers(
        self,
        stamp: rclpy.time.Time,
        next_id,
    ) -> list[Marker]:
        markers = []
        for radius, color, label in [
            (self.minimum_distance, _C_RED, 'min'),
            (self.detection_range, _C_WHITE, 'max'),
        ]:
            m = _make_marker(
                _NS_RANGE, next_id(), Marker.LINE_STRIP,
                self.source_frame, stamp, 0.15, color,
                scale_x=0.015,
            )
            segs = 48
            for i in range(segs + 1):
                a = 2.0 * math.pi * i / segs
                p = Point()
                p.x, p.y, p.z = radius * math.cos(a), radius * math.sin(a), 0.0
                m.points.append(p)
        return markers

    def _make_robot_frame_marker(
        self,
        stamp: rclpy.time.Time,
        next_id,
    ) -> Marker:
        m = _make_marker(
            _NS_ROBOT, next_id(), Marker.ARROW,
            self.source_frame, stamp, 0.15, _C_MAGENTA,
            scale_x=0.5, scale_y=0.04, scale_z=0.04,
        )
        m.points.append(Point(x=0.0, y=0.0, z=0.0))
        m.points.append(Point(x=0.5, y=0.0, z=0.0))
        return m

    def _make_obstacle_debug_markers(
        self,
        body_center: tuple[float, float],
        body_corners: list[tuple[float, float]],
        center_distance: float,
        filter_reason: str | None,
        topic: str,
        stamp: rclpy.time.Time,
        next_id,
    ) -> list[Marker]:
        markers = []
        base_id = hash((topic, body_center)) & 0xFFFF

        if filter_reason == 'too_close':
            color, label = _C_RED, 'too_close'
        elif filter_reason == 'too_far':
            color, label = _C_GREY, 'too_far'
        elif filter_reason == 'out_of_fov':
            color, label = _C_YELLOW, 'out_of_fov'
        else:
            color, label = _C_GREEN, 'detected'

        # Center dot
        center = _make_marker(
            _NS_CENTER, base_id, Marker.SPHERE,
            self.source_frame, stamp, 0.15, color,
            scale_x=0.08, scale_y=0.08, scale_z=0.08,
        )
        center.pose.position.x = body_center[0]
        center.pose.position.y = body_center[1]
        markers.append(center)

        # Bounding box (world frame corners projected to body frame)
        box = _make_marker(
            _NS_OBSTACLE, base_id, Marker.LINE_STRIP,
            self.source_frame, stamp, 0.15, color,
            scale_x=0.02,
        )
        for vx, vy in body_corners:
            box.points.append(Point(x=vx, y=vy, z=0.0))
        # Close the loop
        if body_corners:
            box.points.append(Point(
                x=body_corners[0][0], y=body_corners[0][1], z=0.0
            ))
        markers.append(box)

        # Vertex dots (colored by vertex FOV check)
        for vi, (vx, vy) in enumerate(body_corners):
            vd = math.hypot(vx, vy)
            va = math.atan2(vy, vx)
            half_fov = self.field_of_view / 2.0
            v_visible = (
                vd <= self.detection_range
                and vd >= self.minimum_distance
                and abs(va) <= half_fov
            )
            vcolor = _C_GREEN if v_visible else _C_RED
            vm = _make_marker(
                _NS_VERTICES, base_id * 10 + vi, Marker.SPHERE,
                self.source_frame, stamp, 0.15, vcolor,
                scale_x=0.04, scale_y=0.04, scale_z=0.04,
            )
            vm.pose.position.x = vx
            vm.pose.position.y = vy
            markers.append(vm)

        # Text label with distance/angle info
        if filter_reason is not None:
            label_m = _make_marker(
                _NS_FILTERED, base_id, Marker.TEXT_VIEW_FACING,
                self.source_frame, stamp, 0.15, color,
                scale_x=0.12, scale_y=0.12, scale_z=0.12,
            )
            label_m.pose.position.x = body_center[0]
            label_m.pose.position.y = body_center[1] + 0.2
            label_m.text = f'{label} d={center_distance:.2f}m'
            markers.append(label_m)

        return markers

    def _quaternion_to_yaw(self, quaternion) -> float:
        siny_cosp = 2.0 * (quaternion.w * quaternion.z)
        cosy_cosp = 1.0 - 2.0 * (quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MockObstacleSensor()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
