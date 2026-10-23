from geometry_msgs.msg import Pose


class ObstacleRuntimeState:
    def __init__(self) -> None:
        self.latest_pose: Pose | None = None
        self.current_waypoint_index: int = 0
        self.path_direction: int = 1  # +1 forward along path, -1 reverse (oscillation)
