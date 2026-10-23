#!/usr/bin/env python3
"""Publish a static OccupancyGrid (mock costmap) from a black/white image.

This is a deterministic stand-in for the sensor-built ``/global_costmap/costmap``
so the OGM->Lanelet2 student task can be developed without running Gazebo: the
same image always produces the exact same grid.

Pixel mapping (default): white pixels -> occupied (100), black -> free (0). The
grid is published *latched* (TRANSIENT_LOCAL + RELIABLE) on the same topic and in
the same ``nav_msgs/OccupancyGrid`` format the Nav2 global costmap uses, so any
subscriber written against the real costmap works unchanged.

Parameters
----------
image_path        path to the source image (PNG/JPG, read as grayscale)
costmap_topic     output topic (default /global_costmap/costmap)
frame_id          grid frame (default map)
resolution        metres per cell / pixel (default 0.05)
center_origin     if true, place (0,0) at the image centre (default true)
origin_x/origin_y  origin used when center_origin is false (default 0.0)
map_size_x/map_size_y  if >0, resample so the grid spans exactly this many
                  metres with square cells (used to match a Gazebo plane the
                  texture is stretched onto); 0 keeps native 1px=1cell
occupied_is_white  white=occupied if true, else black=occupied (default true)
occupied_threshold grayscale cutoff 0-255 for "white" (default 128)
publish_rate      republish rate in Hz; <=0 publishes once (default 1.0)
"""

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


class MockCostmapPublisher(Node):
    def __init__(self):
        super().__init__("mock_costmap_publisher")

        self.declare_parameter("image_path", "")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("center_origin", True)
        self.declare_parameter("origin_x", 0.0)
        self.declare_parameter("origin_y", 0.0)
        self.declare_parameter("map_size_x", 0.0)
        self.declare_parameter("map_size_y", 0.0)
        self.declare_parameter("occupied_is_white", True)
        self.declare_parameter("occupied_threshold", 128)
        self.declare_parameter("publish_rate", 1.0)

        image_path = self.get_parameter("image_path").value
        topic = self.get_parameter("costmap_topic").value
        rate = float(self.get_parameter("publish_rate").value)

        if not image_path:
            raise RuntimeError("Parameter 'image_path' is required")

        # Build the grid once -- deterministic: same image -> same grid.
        self._grid = self._build_grid(image_path)

        # Latched QoS so late subscribers (RViz, student nodes) still get it,
        # matching how Nav2 publishes the real costmap.
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(OccupancyGrid, topic, qos)

        self._publish()  # publish immediately (latched)
        if rate > 0.0:
            self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"Mock costmap {self._grid.info.width}x{self._grid.info.height} @ "
            f"{self._grid.info.resolution:.3f} m on '{topic}' "
            f"(frame={self._grid.header.frame_id}) from {image_path}"
        )

    def _build_grid(self, image_path: str) -> OccupancyGrid:
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f"Could not read image: {image_path}")

        resolution = float(self.get_parameter("resolution").value)
        threshold = int(self.get_parameter("occupied_threshold").value)
        occupied_is_white = bool(self.get_parameter("occupied_is_white").value)

        # Optionally resample to a fixed physical extent with square cells. This
        # is how we match a Gazebo ground plane that stretches a non-square image
        # onto a square plane. INTER_NEAREST keeps the lines crisp/binary.
        map_size_x = float(self.get_parameter("map_size_x").value)
        map_size_y = float(self.get_parameter("map_size_y").value)
        if map_size_x > 0.0 and map_size_y > 0.0:
            cols = max(1, round(map_size_x / resolution))
            rows = max(1, round(map_size_y / resolution))
            img = cv2.resize(img, (cols, rows), interpolation=cv2.INTER_NEAREST)

        height, width = img.shape

        # OccupancyGrid is row-major starting at the origin (bottom-left), so
        # row 0 is the *bottom*. Images are top-left origin, so flip vertically
        # to keep the published map looking like the image.
        img = np.flipud(img)

        white = img >= threshold
        occupied = white if occupied_is_white else ~white
        data = np.where(occupied, 100, 0).astype(np.int8)

        grid = OccupancyGrid()
        grid.header.frame_id = self.get_parameter("frame_id").value
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin = self._origin(width, height, resolution)
        grid.data = data.flatten().tolist()
        return grid

    def _origin(self, width: int, height: int, resolution: float) -> Pose:
        pose = Pose()
        if bool(self.get_parameter("center_origin").value):
            pose.position.x = -(width * resolution) / 2.0
            pose.position.y = -(height * resolution) / 2.0
        else:
            pose.position.x = float(self.get_parameter("origin_x").value)
            pose.position.y = float(self.get_parameter("origin_y").value)
        pose.orientation.w = 1.0
        return pose

    def _publish(self):
        self._grid.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._grid)


def main(args=None):
    rclpy.init(args=args)
    node = MockCostmapPublisher()
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
