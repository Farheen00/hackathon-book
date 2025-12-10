#!/usr/bin/env python3

"""
Python-ROS Bridge Example
This example demonstrates how to bridge Python AI agents with ROS controllers.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
import numpy as np
import time


class PythonROSBridge(Node):
    """
    Demonstrates bridging Python AI algorithms with ROS controllers.
    This example implements a simple obstacle avoidance algorithm.
    """

    def __init__(self):
        super().__init__('python_ros_bridge')

        # 1. Sensor data subscription
        self.laser_subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)

        # 2. Command publisher to robot
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # 3. AI decision publisher (for monitoring)
        self.decision_publisher = self.create_publisher(String, 'ai_decision', 10)

        # 4. Store latest sensor data
        self.latest_scan = None

        # 5. AI algorithm parameters
        self.safety_distance = 0.5  # meters
        self.forward_speed = 0.3    # m/s
        self.turn_speed = 0.5       # rad/s

        # 6. Control timer for AI decision loop
        self.control_timer = self.create_timer(0.1, self.ai_control_loop)

        self.get_logger().info('Python-ROS Bridge node initialized')

    def laser_callback(self, msg):
        """Process laser scan data from robot."""
        self.latest_scan = msg
        self.get_logger().debug(f'Received laser scan with {len(msg.ranges)} points')

    def ai_control_loop(self):
        """Main AI control loop that processes sensor data and makes decisions."""
        if self.latest_scan is None:
            return

        # 1. Process sensor data
        obstacles = self.detect_obstacles(self.latest_scan)

        # 2. Make AI decision based on processed data
        decision = self.make_ai_decision(obstacles)

        # 3. Convert decision to robot command
        command = self.decision_to_command(decision)

        # 4. Publish command to robot
        if command is not None:
            self.cmd_publisher.publish(command)

        # 5. Publish decision for monitoring
        decision_msg = String()
        decision_msg.data = decision
        self.decision_publisher.publish(decision_msg)

    def detect_obstacles(self, scan_msg):
        """Detect obstacles from laser scan data."""
        obstacles = []

        # Check for obstacles in front (forward 30 degrees)
        front_start_idx = len(scan_msg.ranges) // 2 - 15
        front_end_idx = len(scan_msg.ranges) // 2 + 15

        for i in range(max(0, front_start_idx), min(len(scan_msg.ranges), front_end_idx)):
            distance = scan_msg.ranges[i]
            if 0 < distance < self.safety_distance:
                obstacles.append((i, distance))

        # Check left and right sectors for navigation
        left_sector = scan_msg.ranges[45:135]  # Left 90 degrees
        right_sector = scan_msg.ranges[225:315]  # Right 90 degrees

        left_clear = all(d > self.safety_distance or d == float('inf') for d in left_sector if d > 0)
        right_clear = all(d > self.safety_distance or d == float('inf') for d in right_sector if d > 0)

        return {
            'front_obstacles': obstacles,
            'left_clear': left_clear,
            'right_clear': right_clear,
            'front_avg': np.mean([d for d in scan_msg.ranges[front_start_idx:front_end_idx]
                                 if 0 < d < float('inf')])
        }

    def make_ai_decision(self, obstacles):
        """AI decision-making algorithm."""
        if len(obstacles['front_obstacles']) > 0:
            # Obstacle detected in front, need to turn
            if obstacles['left_clear'] and obstacles['right_clear']:
                # Both sides clear, choose based on more space
                if self.is_left_more_clear(obstacles):
                    return "turn_left"
                else:
                    return "turn_right"
            elif obstacles['left_clear']:
                return "turn_left"
            elif obstacles['right_clear']:
                return "turn_right"
            else:
                # No clear path, turn around
                return "turn_around"
        else:
            # No obstacles ahead, go forward
            return "forward"

    def is_left_more_clear(self, obstacles):
        """Determine if left side has more clearance than right."""
        # Simplified logic - in practice, you'd do more sophisticated analysis
        return True  # Default to left for this example

    def decision_to_command(self, decision):
        """Convert AI decision to robot command."""
        cmd = Twist()

        if decision == "forward":
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
        elif decision == "turn_left":
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
        elif decision == "turn_right":
            cmd.linear.x = 0.0
            cmd.angular.z = -self.turn_speed
        elif decision == "turn_around":
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed * 1.5  # Faster turn
        elif decision == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # Unknown decision, stop for safety
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        return cmd


class AdvancedPythonROSBridge(Node):
    """
    Advanced example showing more sophisticated AI integration.
    This implements a simple learning algorithm for path planning.
    """

    def __init__(self):
        super().__init__('advanced_python_ros_bridge')

        # Subscriptions
        self.laser_subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)

        # Publishers
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Internal state
        self.latest_scan = None
        self.learning_rate = 0.1
        self.discount_factor = 0.9
        self.exploration_rate = 0.2

        # Simple Q-table for learning
        self.q_table = {}

        # Control timer
        self.control_timer = self.create_timer(0.05, self.learning_control_loop)

    def laser_callback(self, msg):
        """Process laser scan data."""
        self.latest_scan = msg

    def learning_control_loop(self):
        """Advanced control loop with learning capabilities."""
        if self.latest_scan is None:
            return

        # Discretize the state from sensor data
        state = self.discretize_state(self.latest_scan)

        # Choose action using epsilon-greedy strategy
        action = self.choose_action(state)

        # Execute action
        cmd = self.action_to_command(action)
        self.cmd_publisher.publish(cmd)

        # In a real implementation, you would also handle rewards here
        # and update the Q-table based on the state transition

    def discretize_state(self, scan_msg):
        """Convert continuous sensor data to discrete state representation."""
        ranges = np.array(scan_msg.ranges)

        # Divide scan into sectors: front, left, right, back
        front_avg = np.mean(ranges[315:360]) if len(ranges) > 315 else float('inf')
        front_avg += np.mean(ranges[0:45]) if len(ranges) > 45 else 0
        front_avg /= 2

        left_avg = np.mean(ranges[45:135]) if len(ranges) > 135 else float('inf')
        right_avg = np.mean(ranges[225:315]) if len(ranges) > 315 else float('inf')
        back_avg = np.mean(ranges[135:225]) if len(ranges) > 225 else float('inf')

        # Discretize distances into categories
        def discretize_distance(d):
            if d < 0.3:
                return 0  # Very close
            elif d < 0.7:
                return 1  # Close
            elif d < 1.5:
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
        """Choose action using epsilon-greedy strategy."""
        if np.random.random() < self.exploration_rate:
            # Explore: random action
            return np.random.choice(['forward', 'left', 'right', 'backward'])
        else:
            # Exploit: best known action
            if state not in self.q_table:
                self.q_table[state] = {'forward': 0, 'left': 0, 'right': 0, 'backward': 0}

            return max(self.q_table[state], key=self.q_table[state].get)

    def action_to_command(self, action):
        """Convert action to Twist command."""
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


def main_basic(args=None):
    """Run the basic Python-ROS bridge example."""
    rclpy.init(args=args)
    node = PythonROSBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_advanced(args=None):
    """Run the advanced Python-ROS bridge example."""
    rclpy.init(args=args)
    node = AdvancedPythonROSBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import sys

    print("Python-ROS Bridge Examples")
    print("Run with 'python3 python_ros_bridge.py basic' for basic example")
    print("Run with 'python3 python_ros_bridge.py advanced' for advanced example")

    if len(sys.argv) > 1:
        if sys.argv[1] == 'basic':
            main_basic()
        elif sys.argv[1] == 'advanced':
            main_advanced()
    else:
        main_basic()  # Default to basic example