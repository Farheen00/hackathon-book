---
sidebar_position: 2
---

# Introduction to rclpy

## What is rclpy?

**rclpy** is the Python client library for ROS 2. It provides a Python API that allows you to create ROS 2 nodes, publish and subscribe to topics, provide and call services, and interact with other ROS 2 concepts. rclpy is built on top of the ROS Client Library (rcl) and the DDS (Data Distribution Service) implementation.

### Key Features of rclpy:

- **Native Python**: Full integration with Python's ecosystem
- **Asynchronous Support**: Support for both synchronous and asynchronous programming
- **Type Safety**: Integration with ROS 2 message types for type safety
- **Resource Management**: Automatic cleanup of ROS 2 resources
- **Cross-Language Compatibility**: Works seamlessly with nodes written in other languages (C++, etc.)

## Installing and Setting Up rclpy

rclpy comes pre-installed with ROS 2, so no additional installation is required. However, you need to source your ROS 2 environment:

```bash
# For ROS 2 Humble Hawksbill
source /opt/ros/humble/setup.bash

# For ROS 2 Iron Irwini
source /opt/ros/iron/setup.bash
```

## Basic rclpy Node Structure

Here's the fundamental structure of an rclpy node:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Initialize publishers, subscribers, services, etc. here

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Asynchronous Programming with rclpy

rclpy supports both synchronous and asynchronous programming patterns:

### Synchronous Example:

```python
import rclpy
from rclpy.node import Node

class SyncNode(Node):
    def __init__(self):
        super().__init__('sync_node')
        self.get_logger().info('Synchronous node started')

def main(args=None):
    rclpy.init(args=args)
    node = SyncNode()

    try:
        rclpy.spin(node)  # Blocks until shutdown
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Asynchronous Example:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import time

class AsyncNode(Node):
    def __init__(self):
        super().__init__('async_node')
        self.get_logger().info('Asynchronous node started')

        # Example of async operation
        self.timer = self.create_timer(1.0, self.async_callback)

    def async_callback(self):
        self.get_logger().info('Timer callback executed')

def main(args=None):
    rclpy.init(args=args)
    node = AsyncNode()

    # Use MultiThreadedExecutor for async operations
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Working with Parameters

rclpy allows you to define and use parameters that can be configured at runtime:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('max_velocity', 1.0)

        # Get parameter values
        self.frequency = self.get_parameter('frequency').value
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value

        self.get_logger().info(f'Configured: freq={self.frequency}, name={self.robot_name}, max_vel={self.max_velocity}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Lifecycle Management

rclpy supports lifecycle nodes for complex state management:

```python
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

class LifecycleExample(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_node')
        self.get_logger().info('Lifecycle node created')

    def on_configure(self, state):
        self.get_logger().info('Configuring lifecycle node')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating lifecycle node')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating lifecycle node')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up lifecycle node')
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    node = LifecycleExample()

    # Lifecycle nodes need to be configured and activated
    node.configure()
    node.activate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.deactivate()
        node.cleanup()
        rclpy.shutdown()
```

## Importing Message Types

rclpy works with ROS 2 message types from various packages:

```python
# Common message types
from std_msgs.msg import String, Int32, Float64
from geometry_msgs.msg import Twist, Pose, Point
from sensor_msgs.msg import LaserScan, Image, JointState
from nav_msgs.msg import Odometry

# Service types
from std_srvs.srv import SetBool, Trigger
from example_interfaces.srv import AddTwoInts
```

## Error Handling and Logging

Proper error handling is crucial in rclpy nodes:

```python
import rclpy
from rclpy.node import Node

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')

        try:
            # Initialize node components
            self.publisher = self.create_publisher(String, 'topic', 10)
            self.get_logger().info('Node initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize node: {e}')
            raise

    def safe_operation(self):
        try:
            # Perform operation that might fail
            result = self.perform_risky_operation()
            return result
        except Exception as e:
            self.get_logger().error(f'Operation failed: {e}')
            return None

    def perform_risky_operation(self):
        # Simulate some operation
        return "success"

def main(args=None):
    rclpy.init(args=args)
    node = RobustNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Performance Considerations

When working with rclpy, consider these performance tips:

1. **Use appropriate QoS settings** for your application's needs
2. **Minimize message copying** by using efficient data structures
3. **Use appropriate executor types** (MultiThreadedExecutor vs SingleThreadedExecutor)
4. **Manage timers efficiently** to avoid unnecessary CPU usage
5. **Consider using intra-process communication** for high-frequency data exchange

## Debugging rclpy Nodes

Useful debugging techniques for rclpy:

- Use `self.get_logger().debug()` for detailed debugging information
- Monitor topics with `ros2 topic echo`
- Check node status with `ros2 node info`
- Use `rclpy.validate_full_topic_name()` to validate topic names

## Summary

rclpy provides the essential Python interface to ROS 2, enabling seamless integration of Python-based applications with the ROS 2 ecosystem. Understanding rclpy fundamentals is crucial for creating effective Python-to-ROS bridges that can connect AI agents with robotic systems.

In the next section, we'll explore advanced publisher-subscriber patterns that are commonly used in AI-robot integration scenarios.