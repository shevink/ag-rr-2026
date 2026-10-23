import math
from geometry_msgs.msg import Pose, PoseArray


def pose_from_posearray(msg: PoseArray) -> Pose | None:
    # msg.poses behaves as a set and doesn't support indexing — use an iterator
    return next(iter(msg.poses)) if msg.poses else None


def distance(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(bx - ax, by - ay)
