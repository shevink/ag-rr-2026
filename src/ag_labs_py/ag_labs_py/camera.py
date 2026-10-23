import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class VirtualCameraPublisher(Node):
    """Virtual camera node for the simulated RosRacer.

    Instead of opening a USB device (as the real hardware node does), this node
    relays the raw image rendered by the Gazebo camera sensor (delivered to ROS
    by the ros_gz camera bridge) onto the canonical camera topic. The image is
    forwarded uncompressed (sensor_msgs/Image) so downstream nodes such as the
    IPM node consume raw frames.

    Topology (per camera):
        Gazebo sensor -> ros_gz bridge -> <camera>/image_gz  (input_topic)
                                       -> camera node
                                       -> <camera>/image_raw (output_topic)
    """

    def __init__(self):
        super().__init__('camera_publisher_node')

        self.declare_parameter('camera_name', 'camera_high')
        self.camera_name = str(self.get_parameter('camera_name').value)

        # Defaults derived from the camera name; overridable per node.
        self.declare_parameter('input_topic', f'/{self.camera_name}/image_gz')
        self.declare_parameter('output_topic', f'/{self.camera_name}/image_raw')
        self.declare_parameter('frame_id', f'{self.camera_name}_link')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.publisher_ = self.create_publisher(Image, output_topic, 10)
        self.subscription = self.create_subscription(
            Image, input_topic, self.image_callback, 10)

        self.get_logger().info(
            f'Virtual camera "{self.camera_name}" relaying {input_topic} -> '
            f'{output_topic}'
        )

    def image_callback(self, msg: Image):
        # Stamp with the configured frame so downstream TF lookups are stable.
        msg.header.frame_id = self.frame_id
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = VirtualCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
