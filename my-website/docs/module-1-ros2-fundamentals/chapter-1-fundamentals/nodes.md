---
sidebar_position: 2
---

# Understanding ROS 2 Nodes

## What is a Node?

A **Node** is the fundamental building block of a ROS 2 system. It's a process that performs computation and is the basic unit of executable code in ROS 2. Nodes are designed to be modular, allowing you to break down complex robotic applications into smaller, manageable pieces.

### Key Characteristics of Nodes:

- **Modularity**: Each node should have a single, well-defined purpose
- **Communication**: Nodes communicate with each other through topics, services, and actions
- **Distributed**: Nodes can run on the same machine or across different machines
- **Language Agnostic**: Nodes can be written in different programming languages (C++, Python, etc.)

## Creating a Simple Node in Python

Let's create a simple ROS 2 node using Python and the `rclpy` client library:

```python
# simple_node.py
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Simple Node has been started')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()

    # Keep the node running
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

### Breaking Down the Code:

1. **Import Statements**: We import `rclpy` and `Node` from the ROS 2 Python client library
2. **Node Class**: We create a class that inherits from `Node`
3. **Initialization**: In the constructor, we call the parent constructor with a node name
4. **Logging**: We use `get_logger().info()` to output messages
5. **Main Function**: Initializes ROS 2, creates the node, and spins it to keep it running

## Running the Node

To run this node:

1. Save the code as `simple_node.py`
2. Make sure your ROS 2 environment is sourced:
   ```bash
   source /opt/ros/humble/setup.bash  # For Humble
   # OR
   source /opt/ros/iron/setup.bash    # For Iron
   ```
3. Run the node:
   ```bash
   python3 simple_node.py
   ```

## Node Commands

ROS 2 provides command-line tools to interact with nodes:

- `ros2 node list`: Lists all active nodes
- `ros2 node info <node_name>`: Shows information about a specific node
- `ros2 run <package_name> <executable>`: Runs a node from a package

## Best Practices for Nodes

1. **Single Responsibility**: Each node should have one primary function
2. **Error Handling**: Implement proper error handling and logging
3. **Resource Management**: Clean up resources when the node shuts down
4. **Parameter Configuration**: Use ROS 2 parameters for configurable values
5. **Lifecycle Management**: Consider using lifecycle nodes for complex state management

## Common Node Patterns

### Publisher Node Pattern

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from publisher'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
```

### Subscriber Node Pattern

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

## Summary

Nodes are the basic building blocks of ROS 2 systems. They encapsulate functionality and communicate with other nodes through topics, services, and actions. Understanding nodes is crucial for building modular, maintainable robotic applications.

In the next section, we'll explore how nodes communicate using topics and the publish-subscribe pattern.