---
sidebar_position: 3
---

# Topics and Message Passing

## Understanding Topics

A **Topic** is a communication channel in ROS 2 that enables the publish-subscribe messaging pattern. It allows nodes to send and receive messages without having direct knowledge of each other. This decoupling is one of the key strengths of ROS 2 architecture.

### Key Characteristics of Topics:

- **Unidirectional**: Data flows from publishers to subscribers
- **Anonymous**: Publishers don't know who subscribes, and subscribers don't know who publishes
- **Message-based**: All communication happens through standardized message types
- **Typed**: Each topic has a specific message type that defines the data structure

## Publisher-Subscriber Pattern

The publish-subscribe pattern works as follows:

1. **Publisher**: A node that sends messages to a topic
2. **Topic**: The communication channel
3. **Subscriber**: A node that receives messages from the topic

Multiple publishers can send to the same topic, and multiple subscribers can receive from the same topic.

## Creating a Publisher Node

Here's a complete example of a publisher node:

```python
# publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
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

def main(args=None):
    rclpy.init(args=args)
    publisher_node = PublisherNode()

    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Subscriber Node

Here's a complete example of a subscriber node:

```python
# subscriber_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
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

def main(args=None):
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
    main()
```

## Running Publisher and Subscriber

To run both nodes:

1. Terminal 1 (Publisher):
   ```bash
   python3 publisher_node.py
   ```

2. Terminal 2 (Subscriber):
   ```bash
   python3 subscriber_node.py
   ```

You should see the publisher sending messages and the subscriber receiving them.

## Quality of Service (QoS) Settings

QoS settings control how messages are delivered:

```python
from rclpy.qos import QoSProfile

# Example with custom QoS
qos_profile = QoSProfile(depth=10)
self.publisher_ = self.create_publisher(String, 'chatter', qos_profile)
```

Common QoS settings:
- **Reliability**: Whether messages must be delivered (reliable vs. best effort)
- **Durability**: Whether late-joining subscribers get old messages (transient local vs. volatile)
- **History**: How many messages to keep (keep last N vs. keep all)

## Topic Commands

Useful command-line tools for working with topics:

- `ros2 topic list`: List all active topics
- `ros2 topic echo <topic_name>`: Print messages from a topic
- `ros2 topic info <topic_name>`: Show information about a topic
- `ros2 topic pub <topic_name> <msg_type> <args>`: Publish a message to a topic

Example:
```bash
# Echo messages from the chatter topic
ros2 topic echo /chatter std_msgs/msg/String

# Publish a message to chatter
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from command line'"
```

## Common Message Types

ROS 2 provides many built-in message types:

- **std_msgs**: Basic data types (String, Int32, Float64, etc.)
- **geometry_msgs**: Geometric primitives (Point, Pose, Twist, etc.)
- **sensor_msgs**: Sensor data (LaserScan, Image, JointState, etc.)
- **nav_msgs**: Navigation-related messages (Odometry, Path, etc.)

## Best Practices for Topics

1. **Naming Convention**: Use descriptive, lowercase names with underscores (e.g., `/robot/joint_states`)
2. **Message Types**: Use appropriate message types for your data
3. **QoS Settings**: Choose appropriate QoS settings based on your application needs
4. **Topic Rate**: Don't publish too frequently; consider your application's needs
5. **Error Handling**: Handle connection and disconnection events gracefully

## Advanced Topic Concepts

### Latching Topics

Latching ensures that the last message sent to a topic is saved and delivered to new subscribers:

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

# Create a latched topic
latched_qos = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
self.publisher_ = self.create_publisher(String, 'latched_topic', latched_qos)
```

### Topic Remapping

You can remap topic names when running nodes:

```bash
# Run node with topic remapping
ros2 run package_name node_name --ros-args --remap chatter:=new_chatter
```

## Summary

Topics enable decoupled communication between ROS 2 nodes through the publish-subscribe pattern. Understanding topics is crucial for building distributed robotic systems where different components need to share information. The next section covers services for synchronous request-response communication.