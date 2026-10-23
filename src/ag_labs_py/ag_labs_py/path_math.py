import math
from bisect import bisect_left
from dataclasses import dataclass

import numpy
from scipy.interpolate import CubicSpline


SPLINE_SAMPLE_SPACING_METERS = 0.02

VEHICLE_WIDTH_METERS = 0.20
VEHICLE_LENGTH_METERS = 0.30
WHEELBASE_METERS = 0.26
MAX_STEERING_ANGLE_DEGREES = 15.0
MAX_STEERING_ANGLE_RADIANS = math.radians(MAX_STEERING_ANGLE_DEGREES)
MAX_ALLOWED_CURVATURE = math.tan(MAX_STEERING_ANGLE_RADIANS) / WHEELBASE_METERS


@dataclass(frozen=True)
class PathPoint:
    position_forward: float
    position_left: float
    heading_radians: float = 0.0
    curvature: float = 0.0


@dataclass
class ActivePath:
    points: list[PathPoint]
    sample_distances: list[float]
    is_closed: bool
    target_index: int = 0
    maximum_progress_ratio: float = 0.0
    completed_closed_lap: bool = False
    closest_closed_return_distance: float = float("inf")

    @property
    def length(self) -> float:
        if not self.sample_distances:
            return 0.0
        return self.sample_distances[-1]

    def distance_at(self, index: int) -> float:
        if not self.sample_distances:
            return 0.0
        return self.sample_distances[clamp_index(index, self.points)]

    def index_at_distance(self, target_distance: float) -> int:
        if not self.sample_distances:
            return 0
        index = bisect_left(self.sample_distances, target_distance)
        return clamp_index(index, self.points)

    def remaining_distance(self) -> float:
        return max(0.0, self.length - self.distance_at(self.target_index))

    def progress_ratio(self) -> float:
        if self.length <= 0.0:
            return 0.0
        if self.is_closed:
            return self.maximum_progress_ratio
        return self.distance_at(self.target_index) / self.length


def clamp_index(index: int, values: list) -> int:
    return max(0, min(index, len(values) - 1))


def calculate_path_sample_distances(path_points: list[PathPoint]) -> list[float]:
    sample_distances = [0.0]
    for path_index in range(1, len(path_points)):
        sample_distances.append(
            sample_distances[-1]
            + distance_between(path_points[path_index - 1], path_points[path_index])
        )
    return sample_distances


def smooth_path_with_natural_cubic_splines(
    input_points: list[PathPoint],
) -> list[PathPoint]:
    coordinates = numpy.array(
        [
            (path_point.position_forward, path_point.position_left)
            for path_point in input_points
        ]
    )
    segment_lengths = numpy.hypot(*numpy.diff(coordinates, axis=0).T)
    arc_lengths = numpy.concatenate([[0.0], numpy.cumsum(segment_lengths)])

    total_length = float(arc_lengths[-1])
    if total_length < SPLINE_SAMPLE_SPACING_METERS:
        raise ValueError("Path is shorter than one spline sample")

    endpoint_distance = distance_between(input_points[0], input_points[-1])
    boundary_condition = "natural"
    if endpoint_distance <= 1e-4:
        boundary_condition = "periodic"

    forward_spline = CubicSpline(
        arc_lengths, coordinates[:, 0], bc_type=boundary_condition
    )
    left_spline = CubicSpline(
        arc_lengths, coordinates[:, 1], bc_type=boundary_condition
    )

    sample_count = max(2, int(math.ceil(total_length / SPLINE_SAMPLE_SPACING_METERS)) + 1)
    uniform_sample_arc_lengths = numpy.linspace(0.0, total_length, sample_count)
    sample_arc_lengths = numpy.unique(
        numpy.concatenate((uniform_sample_arc_lengths, arc_lengths))
    )

    forward_positions = forward_spline(sample_arc_lengths)
    left_positions = left_spline(sample_arc_lengths)
    forward_first_derivatives = forward_spline(sample_arc_lengths, 1)
    left_first_derivatives = left_spline(sample_arc_lengths, 1)
    forward_second_derivatives = forward_spline(sample_arc_lengths, 2)
    left_second_derivatives = left_spline(sample_arc_lengths, 2)

    headings = numpy.arctan2(left_first_derivatives, forward_first_derivatives)
    speed_squared = forward_first_derivatives ** 2 + left_first_derivatives ** 2
    curvatures = numpy.where(
        speed_squared < 1e-12,
        numpy.inf,
        (
            forward_first_derivatives * left_second_derivatives
            - left_first_derivatives * forward_second_derivatives
        )
        / speed_squared ** 1.5,
    )

    return [
        PathPoint(
            float(forward_positions[sample_index]),
            float(left_positions[sample_index]),
            float(headings[sample_index]),
            float(curvatures[sample_index]),
        )
        for sample_index in range(len(sample_arc_lengths))
    ]


def maximum_required_steering(path_points: list[PathPoint]) -> tuple[float, float]:
    maximum_curvature = max(abs(path_point.curvature) for path_point in path_points)
    return math.atan(WHEELBASE_METERS * maximum_curvature), maximum_curvature


def pure_pursuit_steering_angle(
    current_position: PathPoint, target_point: PathPoint
) -> float:
    heading_to_target = math.atan2(
        target_point.position_left - current_position.position_left,
        target_point.position_forward - current_position.position_forward,
    )
    heading_error = normalize_angle(heading_to_target - current_position.heading_radians)
    lookahead_distance = max(distance_between(current_position, target_point), 1e-6)
    curvature = 2.0 * math.sin(heading_error) / lookahead_distance
    return math.atan(WHEELBASE_METERS * curvature)


def drop_duplicate_points(path_points: list[PathPoint]) -> list[PathPoint]:
    unique_points: list[PathPoint] = []
    for point in path_points:
        if not unique_points or distance_between(unique_points[-1], point) > 1e-4:
            unique_points.append(point)
    return unique_points


def distance_between(first_point: PathPoint, second_point: PathPoint) -> float:
    return math.hypot(
        second_point.position_forward - first_point.position_forward,
        second_point.position_left - first_point.position_left,
    )


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def normalize_angle(angle_radians: float) -> float:
    return math.remainder(angle_radians, 2.0 * math.pi)
