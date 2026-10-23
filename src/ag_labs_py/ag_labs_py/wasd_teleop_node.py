#!/usr/bin/env python3
"""
ROS2 WASD Teleop Node - Video-game style keyboard control
Works with any window (including Gazebo) that has focus
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pynput import keyboard


class WASDTeleopNode(Node):
    def __init__(self):
        super().__init__('wasd_teleop_node')

        # Publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel/wasd', 10)

        # Velocity parameters
        self.declare_parameter('max_speed', 2.0)  # m/s
        self.declare_parameter('max_steering', 2.0)  # rad/s
        self.declare_parameter('acceleration', 5.0)  # m/s²
        self.declare_parameter('deceleration', 8.0)  # m/s²
        self.declare_parameter('steering_acceleration', 12.0)  # rad/s²
        self.declare_parameter('steering_deceleration', 16.0)  # rad/s²

        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering = self.get_parameter('max_steering').value
        self.acceleration = self.get_parameter('acceleration').value
        self.deceleration = self.get_parameter('deceleration').value
        self.steering_acceleration = self.get_parameter('steering_acceleration').value
        self.steering_deceleration = self.get_parameter('steering_deceleration').value

        # Current velocities
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0

        # Key states (video game style - check if held down)
        self.keys_pressed = {
            'w': False,
            'a': False,
            's': False,
            'd': False,
            'shift': False  # Sprint/boost
        }

        # Start keyboard listener in separate thread
        self.listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        )
        self.listener.start()

        # Timer for publishing commands and updating velocities (50Hz for smooth control)
        self.timer = self.create_timer(0.02, self.update_and_publish)
        self.last_update_time = self.get_clock().now()

        self.get_logger().info('✓ WASD Teleop Node started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  W/S - Forward/Backward')
        self.get_logger().info('  A/D - Turn Left/Right')
        self.get_logger().info('  Shift - Boost (2x speed)')
        self.get_logger().info('  L - Toggle lane assist')
        self.get_logger().info('  ESC - Emergency stop')

    def on_key_press(self, key):
        """Called when a key is pressed"""
        try:
            # Handle character keys
            if hasattr(key, 'char') and key.char is not None:
                key_char = key.char.lower()
                if key_char in ['w', 'a', 's', 'd']:
                    self.keys_pressed[key_char] = True

            # Handle special keys
            if key == keyboard.Key.shift or key == keyboard.Key.shift_r:
                self.keys_pressed['shift'] = True

            # Emergency stop
            if key == keyboard.Key.esc:
                self.emergency_stop()

        except AttributeError:
            pass

    def on_key_release(self, key):
        """Called when a key is released"""
        try:
            # Handle character keys
            if hasattr(key, 'char') and key.char is not None:
                key_char = key.char.lower()
                if key_char in ['w', 'a', 's', 'd']:
                    self.keys_pressed[key_char] = False
                elif key_char == 'l':
                    self.l_pressed = False

            # Handle special keys
            if key == keyboard.Key.shift or key == keyboard.Key.shift_r:
                self.keys_pressed['shift'] = False

        except AttributeError:
            pass

    def emergency_stop(self):
        """Immediately stop the vehicle"""
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        for key_name in self.keys_pressed:
            self.keys_pressed[key_name] = False
        self.get_logger().warn('Emergency stop activated!')

    def update_velocities(self, dt):
        """Update velocities based on currently pressed keys (video game style)"""
        # Calculate target velocities based on pressed keys
        forward = 1.0 if self.keys_pressed['w'] else 0.0
        backward = -1.0 if self.keys_pressed['s'] else 0.0
        left = 1.0 if self.keys_pressed['a'] else 0.0
        right = -1.0 if self.keys_pressed['d'] else 0.0

        # Apply boost if shift is held
        speed_multiplier = 2.0 if self.keys_pressed['shift'] else 1.0

        # Set target velocities
        self.target_linear = (forward + backward) * self.max_speed * speed_multiplier
        self.target_angular = (left + right) * self.max_steering
        if self.target_linear < 0.0:
            self.target_angular *= -1.0

        # Smooth acceleration/deceleration (makes control feel better)
        linear_diff = self.target_linear - self.linear_vel
        angular_diff = self.target_angular - self.angular_vel

        # Choose acceleration or deceleration rate
        linear_rate = self.acceleration if abs(self.target_linear) > abs(self.linear_vel) else self.deceleration
        angular_rate = self.steering_acceleration if abs(self.target_angular) > abs(self.angular_vel) else self.steering_deceleration

        # Apply smooth changes
        if abs(linear_diff) > 0.01:
            step = min(abs(linear_diff), linear_rate * dt)
            self.linear_vel += step if linear_diff > 0 else -step
        else:
            self.linear_vel = self.target_linear

        if abs(angular_diff) > 0.01:
            step = min(abs(angular_diff), angular_rate * dt)
            self.angular_vel += step if angular_diff > 0 else -step
        else:
            self.angular_vel = self.target_angular

    def update_and_publish(self):
        """Update velocities and publish command"""
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        if dt <= 0.0:
            dt = 0.02
        elif dt > 0.1:
            dt = 0.1

        # Update velocities based on key states
        self.update_velocities(dt)

        # Do not keep the WASD input active with a zero command.  Once teleop
        # has decelerated to a stop, the zero-motor source is the mux fallback.
        if self.linear_vel == 0.0 and self.angular_vel == 0.0:
            return

        # Create and publish the active command.
        msg = Twist()
        msg.linear.x = float(self.linear_vel)
        msg.angular.z = float(self.angular_vel)
        self.publisher_.publish(msg)

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info("Shutting down WASD Teleop Node...")
        self.listener.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WASDTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
