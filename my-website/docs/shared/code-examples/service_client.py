#!/usr/bin/env python3

"""
Service Client-Server Example for ROS 2
This example demonstrates the service-client pattern in ROS 2.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceServerNode(Node):
    """Service server that adds two integers."""

    def __init__(self):
        super().__init__('service_server_node')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request received: {request.a} + {request.b} = {response.sum}')
        return response


class ServiceClientNode(Node):
    """Service client that calls the service server."""

    def __init__(self):
        super().__init__('service_client_node')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        return future


def main_server(args=None):
    """Run the service server."""
    rclpy.init(args=args)
    service_server_node = ServiceServerNode()

    try:
        rclpy.spin(service_server_node)
    except KeyboardInterrupt:
        pass
    finally:
        service_server_node.destroy_node()
        rclpy.shutdown()


def main_client(args=None):
    """Run the service client."""
    rclpy.init(args=args)
    service_client_node = ServiceClientNode()

    # Send a request
    future = service_client_node.send_request(2, 3)

    try:
        rclpy.spin_until_future_complete(service_client_node, future)
        response = future.result()
        service_client_node.get_logger().info(f'Result: {response.sum}')
    except KeyboardInterrupt:
        pass
    finally:
        service_client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # This example shows both server and client patterns
    # In practice, you would run each in a separate process
    print("This file contains both service server and client examples.")
    print("Run with 'python3 service_client.py server' for server")
    print("Run with 'python3 service_client.py client' for client")

    import sys
    if len(sys.argv) > 1:
        if sys.argv[1] == 'server':
            main_server()
        elif sys.argv[1] == 'client':
            main_client()