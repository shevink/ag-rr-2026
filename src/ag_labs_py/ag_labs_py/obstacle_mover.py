import yaml
import rclpy
from typing import cast
from rclpy.node import Node
from rclpy.publisher import Publisher
from geometry_msgs.msg import Twist, PoseArray
from ag_labs_py.types import Point, Obstacle, ObstacleRuntimeState
from ag_labs_py.path_utils import pose_from_posearray, distance
from pathlib import Path

CONTROL_PERIOD_SECONDS = 0.01
GOAL_TOLERANCE_METERS = 0.1


class ObstacleMover(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_mover')
        self.obstacles: dict[str, Obstacle] = {}
        self.runtime_states: dict[str, ObstacleRuntimeState] = {}
        self.velocity_publishers: dict[str, Publisher] = {}
        
        self.declare_parameter('config_path', rclpy.Parameter.Type.STRING)
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self._parse_config(config_path)
        self._setup_publishers_and_subscriptions()
        self.create_timer(CONTROL_PERIOD_SECONDS, self._control_loop)  # type: ignore[reportUnknownMemberType]

    def _parse_config(self, config_path_str: str) -> None:
        config_path = Path(config_path_str)
        with open(config_path, 'r') as config_file:
            config = yaml.safe_load(config_file)

        for obstacle_name, obstacle_data in config.get('dynamic', {}).items():
            path_points = [Point(float(point['x']), float(point['y'])) for point in obstacle_data['path']['points']]
            self.obstacles[obstacle_name] = Obstacle(
                oscillation=obstacle_data['oscillation'],
                speed_ms=float(obstacle_data['speed_ms']),
                path_points=path_points,
            )
            self.runtime_states[obstacle_name] = ObstacleRuntimeState()

    def _setup_publishers_and_subscriptions(self) -> None:
        for obstacle_name in self.obstacles:
            self.velocity_publishers[obstacle_name] = self.create_publisher(  # type: ignore[assignment]
                Twist, f'/{obstacle_name}/cmd_vel', 10
            )
            self.create_subscription(  # type: ignore[call-arg]
                PoseArray,
                f'/{obstacle_name}/pose_ground_truth',
                lambda msg, name=obstacle_name: self._on_pose_received(cast(PoseArray, msg), name),
                10,
            )

    def _on_pose_received(self, msg: PoseArray, obstacle_name: str) -> None:
        pose = pose_from_posearray(msg)
        if pose is not None:
            self.runtime_states[obstacle_name].latest_pose = pose

    def _control_loop(self) -> None:
        for obstacle_name, obstacle in self.obstacles.items():
            self._step_obstacle(obstacle_name, obstacle)

    def _step_obstacle(self, obstacle_name: str, obstacle: Obstacle) -> None:
        runtime_state = self.runtime_states[obstacle_name]
        if runtime_state.latest_pose is None:
            return

        current_position_x = cast(float, runtime_state.latest_pose.position.x)  # type: ignore[union-attr]
        current_position_y = cast(float, runtime_state.latest_pose.position.y)  # type: ignore[union-attr]
        target_waypoint = obstacle.path_points[runtime_state.current_waypoint_index]
        distance_to_waypoint = distance(
            current_position_x, current_position_y,
            target_waypoint.x, target_waypoint.y,
        )

        velocity_command = Twist()

        if distance_to_waypoint > GOAL_TOLERANCE_METERS:
            velocity_command.linear.x = obstacle.speed_ms * (target_waypoint.x - current_position_x) / distance_to_waypoint  # type: ignore[union-attr]
            velocity_command.linear.y = obstacle.speed_ms * (target_waypoint.y - current_position_y) / distance_to_waypoint  # type: ignore[union-attr]
        else:
            next_waypoint_index = runtime_state.current_waypoint_index + runtime_state.path_direction
            if 0 <= next_waypoint_index < len(obstacle.path_points):
                runtime_state.current_waypoint_index = next_waypoint_index
            elif obstacle.oscillation:
                runtime_state.path_direction *= -1
                runtime_state.current_waypoint_index += runtime_state.path_direction
            else:
                runtime_state.current_waypoint_index = 0

        self.velocity_publishers[obstacle_name].publish(velocity_command)  # type: ignore[union-attr]


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ObstacleMover()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
