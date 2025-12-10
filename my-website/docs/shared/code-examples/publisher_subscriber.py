#!/usr/bin/env python3

"""
Publisher-Subscriber Example for ROS 2
This example demonstrates the basic publisher-subscriber pattern in ROS 2.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PublisherNode(Node):
    """Publisher node that sends messages to a topic."""

    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


class SubscriberNode(Node):
    """Subscriber node that receives messages from a topic."""

    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'chatter',  # Topic name must match publisher
            self.listener_callback,
            10)  # QoS profile depth
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main_publisher(args=None):
    """Run the publisher node."""
    rclpy.init(args=args)
    publisher_node = PublisherNode()

    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()


def main_subscriber(args=None):
    """Run the subscriber node."""
    rclpy.init(args=args)
    subscriber_node = SubscriberNode()

    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # This example shows both patterns
    # In practice, you would run each in a separate process
    print("This file contains both publisher and subscriber examples.")
    print("Run with 'python3 publisher_subscriber.py pub' for publisher")
    print("Run with 'python3 publisher_subscriber.py sub' for subscriber")

    import sys
    if len(sys.argv) > 1:
        if sys.argv[1] == 'pub':
            main_publisher()
        elif sys.argv[1] == 'sub':
            main_subscriber()