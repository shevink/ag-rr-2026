"""Geometry utilities for obstacle detection sensor.

Provides functions for computing obstacle corner vertices,
counter-clockwise vertex sorting, and coordinate frame transformations.
All functions operate on (x, y) tuples.
"""

import math


def compute_corners(
    ox: float, oy: float, oyaw: float, half_size: float
) -> list[tuple[float, float]]:
    """Compute the four world-frame corners of a square obstacle.

    Args:
        ox: Obstacle center x in world frame.
        oy: Obstacle center y in world frame.
        oyaw: Obstacle yaw (rotation) in radians.
        half_size: Half the side length of the square obstacle.

    Returns:
        List of four (x, y) corner tuples in CCW order.
    """
    cos_yaw = math.cos(oyaw)
    sin_yaw = math.sin(oyaw)
    corners = []
    for lx, ly in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
        wx = ox + lx * half_size * cos_yaw - ly * half_size * sin_yaw
        wy = oy + lx * half_size * sin_yaw + ly * half_size * cos_yaw
        corners.append((wx, wy))
    return corners


def sort_counter_clockwise(
    points: list[tuple[float, float]]
) -> list[tuple[float, float]]:
    """Sort vertices counter-clockwise by polar angle around centroid.

    Args:
        points: List of (x, y) tuples.

    Returns:
        New list sorted by ascending polar angle (CCW order).
    """
    if len(points) <= 1:
        return points[:]
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    return sorted(points, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


def world_to_body(
    point: tuple[float, float],
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
) -> tuple[float, float]:
    """Transform a world-frame point to the robot's body frame.

    Applies inverse rotation and translation of the robot's pose.

    Args:
        point: World-frame (x, y) tuple.
        robot_x: Robot x position in world frame.
        robot_y: Robot y position in world frame.
        robot_yaw: Robot yaw in radians.

    Returns:
        (x, y) tuple in the robot's body frame.
    """
    dx = point[0] - robot_x
    dy = point[1] - robot_y
    vx = dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw)
    vy = dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)
    return (vx, vy)
