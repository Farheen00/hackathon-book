---
sidebar_position: 4
---

# AI-Robot Integration

## Overview

This section covers the integration of AI agents with robotic systems using the Python-to-ROS bridge. You'll learn how to connect AI algorithms with ROS controllers to create intelligent robotic applications that can perceive, reason, and act in the physical world.

## Architecture of AI-Robot Systems

The typical architecture of an AI-robot system includes:

```
[AI Agent] ←→ [ROS Bridge] ←→ [Robot Controller]
    ↑              ↑              ↑
Sensors ←→ ROS Topics ←→ Actuators
```

The ROS bridge serves as the communication layer between high-level AI algorithms and low-level robot controllers.

## Pattern 1: Perception-Action Loop

A complete perception-action loop using ROS and AI:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import cv2
from cv2 import cv2
from ultralytics import YOLO  # Example AI model

class AIPerceptionAction(Node):
    def __init__(self):
        super().__init__('ai_perception_action')

        # Sensor subscriptions
        self.laser_subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.camera_subscription = self.create_subscription(
            Image, 'camera/image_raw', self.camera_callback, 10)

        # Command publisher
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # AI model initialization
        self.yolo_model = YOLO('yolov8n.pt')  # Load YOLO model
        self.latest_image = None
        self.latest_laser = None

        # Control timer for perception-action loop
        self.timer = self.create_timer(0.1, self.perception_action_loop)

        self.get_logger().info('AI Perception-Action node initialized')

    def laser_callback(self, msg):
        self.latest_laser = np.array(msg.ranges)

    def camera_callback(self, msg):
        # Convert ROS Image to OpenCV format
        # This is a simplified conversion - real implementation would need proper conversion
        self.latest_image = msg  # Store for processing

    def perception_action_loop(self):
        if self.latest_laser is not None and self.latest_image is not None:
            # Process sensor data
            obstacles = self.detect_obstacles(self.latest_laser)
            objects = self.detect_objects_in_image(self.latest_image)

            # Make AI decision based on perception
            command = self.make_decision(obstacles, objects)

            # Execute command
            if command is not None:
                self.cmd_publisher.publish(command)

    def detect_obstacles(self, laser_data):
        # Simple obstacle detection: check for obstacles within 1 meter
        obstacles = []
        for i, distance in enumerate(laser_data):
            if 0.1 < distance < 1.0:  # Valid range with obstacle
                obstacles.append((i, distance))
        return obstacles

    def detect_objects_in_image(self, image_msg):
        # This would convert ROS image to OpenCV format and run AI model
        # Simplified placeholder
        try:
            # Convert ROS Image message to OpenCV format (simplified)
            # In practice, you'd use cv_bridge for proper conversion
            # objects = self.yolo_model(image_cv)
            # return objects
            return []  # Placeholder
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')
            return []

    def make_decision(self, obstacles, objects):
        cmd = Twist()

        # Simple navigation logic
        if len(obstacles) > 0:
            # If obstacles detected, turn away
            cmd.angular.z = 0.5  # Turn left
        else:
            # If no obstacles, move forward
            cmd.linear.x = 0.3

        # Add object-based decisions here
        # For example: if specific object detected, navigate toward it

        return cmd

def main(args=None):
    rclpy.init(args=args)
    node = AIPerceptionAction()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Pattern 2: Behavior Trees with ROS

Implementing behavior trees for complex AI-robot behaviors:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

class BehaviorTreeAI(Node):
    def __init__(self):
        super().__init__('behavior_tree_ai')

        # Subscriptions
        self.laser_subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.emergency_subscription = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_callback, 10)

        # Publishers
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Robot state
        self.latest_scan = None
        self.emergency_stop = False
        self.last_behavior_time = time.time()

        # Behavior timer
        self.timer = self.create_timer(0.05, self.run_behavior_tree)

    def laser_callback(self, msg):
        self.latest_scan = msg

    def emergency_callback(self, msg):
        self.emergency_stop = msg.data

    def run_behavior_tree(self):
        if self.emergency_stop:
            self.execute_stop_behavior()
            return

        if self.latest_scan is None:
            return

        # Behavior tree execution
        if self.is_dangerous_obstacle_ahead():
            self.execute_avoidance_behavior()
        elif self.is_target_detected():
            self.execute_approach_behavior()
        else:
            self.execute_exploration_behavior()

    def is_dangerous_obstacle_ahead(self):
        if self.latest_scan is None:
            return False

        # Check front 30 degrees for obstacles within 0.5m
        front_start = len(self.latest_scan.ranges) // 2 - 15
        front_end = len(self.latest_scan.ranges) // 2 + 15

        for i in range(front_start, front_end):
            if i < len(self.latest_scan.ranges):
                if 0 < self.latest_scan.ranges[i] < 0.5:
                    return True
        return False

    def is_target_detected(self):
        # Placeholder - in real implementation, this would check for specific targets
        # Could be based on camera detection, RFID, etc.
        return False

    def execute_stop_behavior(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_publisher.publish(cmd)

    def execute_avoidance_behavior(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Turn to avoid
        self.cmd_publisher.publish(cmd)

    def execute_approach_behavior(self):
        cmd = Twist()
        cmd.linear.x = 0.3  # Move forward
        cmd.angular.z = 0.0
        self.cmd_publisher.publish(cmd)

    def execute_exploration_behavior(self):
        cmd = Twist()
        cmd.linear.x = 0.2  # Slow forward movement
        cmd.angular.z = 0.0
        self.cmd_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeAI()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Pattern 3: State Machine Integration

Using state machines for AI-robot interaction:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class StateMachineAI(Node):
    def __init__(self):
        super().__init__('state_machine_ai')

        # Subscriptions
        self.laser_subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)

        # Publishers
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, 'ai_state', 10)

        # State machine
        self.states = {
            'SEARCHING': self.searching_behavior,
            'APPROACHING': self.approaching_behavior,
            'AWARENESS': self.awareness_behavior,
            'RETURNING': self.returning_behavior
        }
        self.current_state = 'SEARCHING'
        self.state_start_time = time.time()

        # Robot data
        self.latest_scan = None
        self.target_location = None
        self.return_location = (0.0, 0.0)  # Home position

        # State timer
        self.timer = self.create_timer(0.1, self.state_machine_loop)

    def laser_callback(self, msg):
        self.latest_scan = msg

    def state_machine_loop(self):
        # Execute current state behavior
        if self.current_state in self.states:
            new_state = self.states[self.current_state]()

            # Change state if needed
            if new_state and new_state != self.current_state:
                self.get_logger().info(f'State change: {self.current_state} → {new_state}')
                self.current_state = new_state
                self.state_start_time = time.time()

                # Publish new state
                state_msg = String()
                state_msg.data = self.current_state
                self.state_publisher.publish(state_msg)

    def searching_behavior(self):
        # Simple wall-following for exploration
        cmd = Twist()

        if self.latest_scan:
            # Simple wall following on the right
            right_distances = self.latest_scan.ranges[315:360] + self.latest_scan.ranges[0:45]
            avg_right = sum(right_distances) / len(right_distances)

            if avg_right > 0.8:  # Too far from wall
                cmd.angular.z = -0.3  # Turn right
            elif avg_right < 0.4:  # Too close to wall
                cmd.angular.z = 0.3   # Turn left
            else:
                cmd.linear.x = 0.3   # Go forward

        self.cmd_publisher.publish(cmd)

        # Transition to approaching if target detected (simplified)
        # In real implementation, this would check for specific targets
        if self.is_target_detected():
            return 'APPROACHING'

        return None  # Stay in current state

    def approaching_behavior(self):
        cmd = Twist()
        cmd.linear.x = 0.2  # Approach slowly
        self.cmd_publisher.publish(cmd)

        # Check if close enough to target
        if self.is_at_target():
            return 'AWARENESS'

        # If we've been approaching too long, go back to searching
        if time.time() - self.state_start_time > 30:  # 30 seconds timeout
            return 'SEARCHING'

        return None

    def awareness_behavior(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Rotate to scan surroundings
        self.cmd_publisher.publish(cmd)

        # After 10 seconds of awareness, return home
        if time.time() - self.state_start_time > 10:
            return 'RETURNING'

        return None

    def returning_behavior(self):
        cmd = Twist()
        cmd.linear.x = -0.2  # Go back (simplified - real implementation would navigate to return location)
        self.cmd_publisher.publish(cmd)

        # If back at home position
        if self.is_at_home():
            return 'SEARCHING'

        # If returning takes too long, give up and search
        if time.time() - self.state_start_time > 60:  # 60 seconds timeout
            return 'SEARCHING'

        return None

    def is_target_detected(self):
        # Placeholder - would check for specific targets in real implementation
        return False

    def is_at_target(self):
        # Placeholder - would check distance to target
        return False

    def is_at_home(self):
        # Placeholder - would check if at return location
        return False

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineAI()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Pattern 4: Learning and Adaptation

Implementing basic learning capabilities:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
import pickle
import os

class LearningAI(Node):
    def __init__(self):
        super().__init__('learning_ai')

        # Subscriptions
        self.laser_subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.reward_subscription = self.create_subscription(
            Float32, 'reward', self.reward_callback, 10)

        # Publishers
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Learning components
        self.latest_scan = None
        self.previous_state = None
        self.previous_action = None
        self.q_table = {}  # Simple Q-table for learning
        self.learning_rate = 0.1
        self.discount_factor = 0.9
        self.exploration_rate = 0.2

        # Load previous learning if available
        self.load_q_table()

        # Learning timer
        self.timer = self.create_timer(0.1, self.learning_loop)

    def laser_callback(self, msg):
        self.latest_scan = msg

    def reward_callback(self, msg):
        # Handle external reward signal (e.g., from simulation or human feedback)
        if self.previous_state is not None and self.previous_action is not None:
            self.update_q_value(self.previous_state, self.previous_action, msg.data)

    def learning_loop(self):
        if self.latest_scan is None:
            return

        # Discretize the state (simplified)
        state = self.discretize_state(self.latest_scan)

        # Choose action using epsilon-greedy
        action = self.choose_action(state)

        # Execute action
        cmd = self.action_to_command(action)
        self.cmd_publisher.publish(cmd)

        # Store for next iteration
        self.previous_state = state
        self.previous_action = action

    def discretize_state(self, scan_msg):
        # Simplify laser scan into discrete state representation
        ranges = np.array(scan_msg.ranges)
        # Divide into sectors: front, left, right, back
        front_avg = np.mean(ranges[337:360]) if len(ranges) > 337 else float('inf')
        front_avg += np.mean(ranges[0:23]) if len(ranges) > 23 else 0
        front_avg /= 2

        left_avg = np.mean(ranges[45:135]) if len(ranges) > 135 else float('inf')
        right_avg = np.mean(ranges[225:315]) if len(ranges) > 315 else float('inf')
        back_avg = np.mean(ranges[158:203]) if len(ranges) > 203 else float('inf')

        # Discretize distances into categories
        def discretize_distance(d):
            if d < 0.5:
                return 0  # Very close
            elif d < 1.0:
                return 1  # Close
            elif d < 2.0:
                return 2  # Medium
            else:
                return 3  # Far

        state = (
            discretize_distance(front_avg),
            discretize_distance(left_avg),
            discretize_distance(right_avg),
            discretize_distance(back_avg)
        )
        return state

    def choose_action(self, state):
        # Epsilon-greedy action selection
        if np.random.random() < self.exploration_rate:
            # Explore: random action
            return np.random.choice(['forward', 'left', 'right', 'backward'])
        else:
            # Exploit: best known action
            if state not in self.q_table:
                self.q_table[state] = {'forward': 0, 'left': 0, 'right': 0, 'backward': 0}

            return max(self.q_table[state], key=self.q_table[state].get)

    def action_to_command(self, action):
        cmd = Twist()
        if action == 'forward':
            cmd.linear.x = 0.3
        elif action == 'left':
            cmd.angular.z = 0.5
        elif action == 'right':
            cmd.angular.z = -0.5
        elif action == 'backward':
            cmd.linear.x = -0.3
        return cmd

    def update_q_value(self, state, action, reward):
        if state not in self.q_table:
            self.q_table[state] = {'forward': 0, 'left': 0, 'right': 0, 'backward': 0}

        # Get current Q-value
        current_q = self.q_table[state][action]

        # Calculate max Q-value for next state (simplified - assumes same state)
        next_max_q = max(self.q_table[state].values()) if state in self.q_table else 0

        # Update Q-value using Q-learning formula
        new_q = current_q + self.learning_rate * (
            reward + self.discount_factor * next_max_q - current_q
        )
        self.q_table[state][action] = new_q

    def load_q_table(self):
        if os.path.exists('q_table.pkl'):
            try:
                with open('q_table.pkl', 'rb') as f:
                    self.q_table = pickle.load(f)
                self.get_logger().info('Loaded Q-table from file')
            except Exception as e:
                self.get_logger().error(f'Error loading Q-table: {e}')

    def save_q_table(self):
        try:
            with open('q_table.pkl', 'wb') as f:
                pickle.dump(self.q_table, f)
            self.get_logger().info('Saved Q-table to file')
        except Exception as e:
            self.get_logger().error(f'Error saving Q-table: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LearningAI()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_q_table()  # Save learning before shutdown
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Safety Considerations

When integrating AI with robots, safety is paramount:

### 1. Emergency Stop Systems
- Implement hard emergency stops that override AI decisions
- Use safety-rated hardware where possible
- Monitor for dangerous situations and intervene

### 2. Fail-Safe Behaviors
- Define safe default behaviors when AI fails
- Implement watchdog timers to detect stuck states
- Use multiple sensor modalities for redundancy

### 3. Validation and Testing
- Test AI behaviors in simulation before real deployment
- Implement logging for debugging and analysis
- Use formal verification where possible

## Performance Optimization

### 1. Real-time Constraints
- Use appropriate ROS QoS settings for time-critical data
- Consider using Real-time ROS (rtr) for safety-critical applications
- Optimize AI model inference for real-time performance

### 2. Resource Management
- Monitor CPU and memory usage
- Use efficient data structures
- Consider model quantization for embedded deployment

### 3. Communication Efficiency
- Use appropriate message rates
- Consider data compression for high-bandwidth sensors
- Use intra-process communication when possible

## Summary

AI-robot integration enables the creation of intelligent robotic systems that can perceive, reason, and act autonomously. The patterns covered in this section provide a foundation for building sophisticated AI-robot applications, from simple perception-action loops to complex learning systems.

Remember that safety, reliability, and validation are crucial when deploying AI-robot systems, especially in real-world environments. The Python-to-ROS bridge provides the essential communication layer that makes this integration possible, enabling you to leverage the rich Python AI ecosystem with ROS's robotic capabilities.

In the next chapter, we'll explore how to model humanoid robots using URDF, which is essential for simulation and control of complex robotic systems.