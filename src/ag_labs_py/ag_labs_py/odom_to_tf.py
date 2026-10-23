#!/usr/bin/env python3
"""
ROS 2 node that broadcasts a TF transform from an odometry topic.

Subscribes to a configurable odometry topic, extracts the pose
(position + orientation), and broadcasts it as a TF transform
from a configurable parent frame to a configurable child frame.
"""

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomToTf(Node):
    def __init__(self):
        super().__init__("odom_to_tf")

        # Declare parameters with defaults
        self.declare_parameter("odom_topic", "/racer/odom")
        self.declare_parameter("parent_frame", "racer/odom")
        self.declare_parameter("child_frame", "racer/base_footprint")

        odom_topic = self.get_parameter("odom_topic").value
        parent_frame = self.get_parameter("parent_frame").value
        child_frame = self.get_parameter("child_frame").value

        self._parent_frame = parent_frame
        self._child_frame = child_frame

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        self.get_logger().info(
            f"Publishing TF: {parent_frame} -> {child_frame} from topic {odom_topic}"
        )

    def _odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._parent_frame
        t.child_frame_id = self._child_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
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
