---
sidebar_position: 3
---

# Advanced Publisher-Subscriber Patterns

## Overview

In this section, we'll explore advanced publisher-subscriber patterns that are commonly used when bridging Python AI agents with ROS controllers. These patterns go beyond the basic examples and address real-world scenarios in robotics applications.

## Pattern 1: Sensor Data Aggregation

When working with AI agents, you often need to aggregate data from multiple sensors:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np

class SensorAggregator(Node):
    def __init__(self):
        super().__init__('sensor_aggregator')

        # Subscriptions for different sensor types
        self.laser_subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.joint_subscription = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)

        # Publisher for aggregated data
        self.aggregated_publisher = self.create_publisher(
            Float32MultiArray, 'sensor_aggregated', 10)

        # Store latest sensor data
        self.latest_laser = None
        self.latest_joints = None

    def laser_callback(self, msg):
        self.latest_laser = np.array(msg.ranges)
        self.publish_aggregated_data()

    def joint_callback(self, msg):
        self.latest_joints = np.array(msg.position)
        self.publish_aggregated_data()

    def publish_aggregated_data(self):
        if self.latest_laser is not None and self.latest_joints is not None:
            # Combine sensor data into a single array
            combined_data = np.concatenate([
                self.latest_laser[:50],  # First 50 laser readings
                self.latest_joints      # All joint positions
            ])

            # Publish aggregated data
            msg = Float32MultiArray()
            msg.data = combined_data.tolist()
            self.aggregated_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorAggregator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Pattern 2: Message Filtering and Processing

Filter sensor data before processing by AI agents:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from collections import deque
import statistics

class SensorFilter(Node):
    def __init__(self):
        super().__init__('sensor_filter')

        self.subscription = self.create_subscription(
            LaserScan, 'raw_scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(
            Float32, 'filtered_distance', 10)

        # For filtering (moving average)
        self.distance_buffer = deque(maxlen=5)

    def scan_callback(self, msg):
        # Get the middle range reading (front of robot)
        if len(msg.ranges) > 0:
            middle_idx = len(msg.ranges) // 2
            raw_distance = msg.ranges[middle_idx]

            # Filter out invalid readings
            if not (raw_distance != raw_distance or  # Check for NaN
                    raw_distance == float('inf') or  # Check for inf
                    raw_distance < msg.range_min or  # Check range bounds
                    raw_distance > msg.range_max):

                self.distance_buffer.append(raw_distance)

                # Calculate moving average
                if len(self.distance_buffer) > 0:
                    filtered_distance = statistics.mean(self.distance_buffer)

                    # Publish filtered result
                    output_msg = Float32()
                    output_msg.data = float(filtered_distance)
                    self.publisher.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Pattern 3: Throttled Publishing

Control the rate at which AI agents receive data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import time

class ThrottledPublisher(Node):
    def __init__(self):
        super().__init__('throttled_publisher')

        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(
            Image, 'camera/image_throttled', 10)

        self.last_publish_time = 0
        self.publish_interval = 0.1  # 10 Hz

    def image_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Only publish if enough time has passed
        if current_time - self.last_publish_time >= self.publish_interval:
            self.publisher.publish(msg)
            self.last_publish_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = ThrottledPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Pattern 4: Message Batching

Batch multiple messages for efficient processing:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from collections import deque

class MessageBatcher(Node):
    def __init__(self):
        super().__init__('message_batcher')

        self.subscription = self.create_subscription(
            Float32, 'sensor_input', self.sensor_callback, 10)
        self.publisher = self.create_publisher(
            Float32MultiArray, 'batched_output', 10)

        self.batch_buffer = deque(maxlen=10)  # Batch size of 10

    def sensor_callback(self, msg):
        self.batch_buffer.append(msg.data)

        # Publish batch when full
        if len(self.batch_buffer) == self.batch_buffer.maxlen:
            batch_msg = Float32MultiArray()
            batch_msg.data = list(self.batch_buffer)
            self.publisher.publish(batch_msg)
            self.batch_buffer.clear()  # Reset for next batch

def main(args=None):
    rclpy.init(args=args)
    node = MessageBatcher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Pattern 5: Conditional Publishing

Publish based on AI agent decisions:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

class ConditionalPublisher(Node):
    def __init__(self):
        super().__init__('conditional_publisher')

        # AI decision input
        self.ai_decision_subscription = self.create_subscription(
            String, 'ai_decision', self.ai_decision_callback, 10)

        # Emergency stop input
        self.emergency_subscription = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_callback, 10)

        # Command output
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.emergency_stop_active = False
        self.last_decision = None

    def ai_decision_callback(self, msg):
        if not self.emergency_stop_active:  # Only if no emergency
            # Process AI decision and convert to robot command
            cmd = self.process_ai_decision(msg.data)
            if cmd:
                self.cmd_publisher.publish(cmd)

    def emergency_callback(self, msg):
        self.emergency_stop_active = msg.data
        if self.emergency_stop_active:
            # Send stop command immediately
            stop_cmd = Twist()
            self.cmd_publisher.publish(stop_cmd)

    def process_ai_decision(self, decision):
        cmd = Twist()
        if decision == "forward":
            cmd.linear.x = 0.5
        elif decision == "turn_left":
            cmd.angular.z = 0.5
        elif decision == "turn_right":
            cmd.angular.z = -0.5
        elif decision == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            return None  # Invalid decision
        return cmd

def main(args=None):
    rclpy.init(args=args)
    node = ConditionalPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Pattern 6: Multi-Topic Synchronization

Synchronize data from multiple topics:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
import threading

class DataSynchronizer(Node):
    def __init__(self):
        super().__init__('data_synchronizer')

        # Create subscribers
        self.laser_sub = Subscriber(self, LaserScan, 'scan')
        self.imu_sub = Subscriber(self, Imu, 'imu/data')

        # Synchronize topics with approximate time sync
        self.ats = ApproximateTimeSynchronizer(
            [self.laser_sub, self.imu_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.ats.registerCallback(self.sync_callback)

        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def sync_callback(self, laser_msg, imu_msg):
        # Process synchronized data
        self.get_logger().info(
            f'Synchronized at: {laser_msg.header.stamp.sec}, '
            f'Laser ranges: {len(laser_msg.ranges)}, '
            f'IMU orientation: {imu_msg.orientation.x}'
        )

        # Send processed command based on synchronized data
        cmd = Twist()
        # Example: use IMU data to adjust command based on robot orientation
        cmd.linear.x = 0.5  # Move forward
        cmd.angular.z = imu_msg.orientation.z * 0.1  # Small correction based on orientation

        self.cmd_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DataSynchronizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Best Practices for Advanced Patterns

### 1. Memory Management
- Use `deque` with `maxlen` for fixed-size buffers
- Clear buffers periodically to prevent memory leaks
- Monitor memory usage in long-running applications

### 2. Thread Safety
- Most rclpy operations are thread-safe, but be careful with shared data
- Use locks for complex shared state
- Consider using `MultiThreadedExecutor` for CPU-intensive processing

### 3. Performance Optimization
- Use appropriate QoS settings for your application
- Avoid unnecessary message copying
- Consider using intra-process communication for high-frequency data

### 4. Error Handling
- Handle sensor data validation (NaN, inf, out-of-range values)
- Implement fallback behaviors when data is unavailable
- Log errors appropriately for debugging

### 5. Real-time Considerations
- Use appropriate timer intervals
- Consider message age when processing time-sensitive data
- Implement watchdogs for safety-critical applications

## Summary

Advanced publisher-subscriber patterns are essential for creating robust AI-robot interfaces. These patterns address real-world challenges such as sensor data aggregation, filtering, synchronization, and conditional publishing. Understanding these patterns will help you build more sophisticated and reliable AI-robot integration systems.

In the next section, we'll explore how to integrate AI agents with robotic systems using these patterns in practical applications.