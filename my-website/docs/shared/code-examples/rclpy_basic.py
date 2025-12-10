#!/usr/bin/env python3

"""
Basic rclpy Example
This example demonstrates fundamental rclpy concepts and patterns.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.parameter import Parameter


class BasicRclpyNode(Node):
    """Demonstrates basic rclpy functionality."""

    def __init__(self):
        super().__init__('basic_rclpy_node')

        # 1. Basic publisher
        self.publisher = self.create_publisher(String, 'basic_topic', 10)

        # 2. Publisher with custom QoS
        qos_profile = QoSProfile(depth=5)
        self.qos_publisher = self.create_publisher(Int32, 'qos_topic', qos_profile)

        # 3. Basic subscriber
        self.subscription = self.create_subscription(
            String, 'input_topic', self.subscription_callback, 10)

        # 4. Timer for periodic publishing
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

        # 5. Declare and use parameters
        self.declare_parameter('example_param', 'default_value')
        self.declare_parameter('frequency', 1.0)

        param_value = self.get_parameter('example_param').value
        self.get_logger().info(f'Parameter value: {param_value}')

        # 6. Create a latched topic (transient local)
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.latched_publisher = self.create_publisher(String, 'latched_topic', latched_qos)

    def timer_callback(self):
        """Callback function called by the timer."""
        msg = String()
        msg.data = f'Hello from basic node: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

        # Publish to QoS topic
        int_msg = Int32()
        int_msg.data = self.counter
        self.qos_publisher.publish(int_msg)

        self.counter += 1

    def subscription_callback(self, msg):
        """Callback function for subscription."""
        self.get_logger().info(f'Received: {msg.data}')

        # Echo the message back with modification
        echo_msg = String()
        echo_msg.data = f'Echo: {msg.data}'
        self.publisher.publish(echo_msg)


def main(args=None):
    """Main function to run the basic rclpy node."""
    rclpy.init(args=args)
    node = BasicRclpyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Caught keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()