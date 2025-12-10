#!/usr/bin/env python3

"""
AI Robot Interface Example
This example demonstrates a complete AI-robot integration system with perception,
decision making, and action execution.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
import numpy as np
import time
from collections import deque
import threading


class AIPerceptionNode(Node):
    """Handles sensor data processing and perception for AI-robot interface."""

    def __init__(self):
        super().__init__('ai_perception_node')

        # Subscriptions for various sensors
        self.laser_subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        # Publisher for processed perception data
        self.perception_publisher = self.create_publisher(
            String, 'perception_data', 10)

        # Internal state
        self.latest_laser = None
        self.latest_odom = None
        self.perception_buffer = deque(maxlen=10)

        # Processing timer
        self.process_timer = self.create_timer(0.1, self.process_perception)

    def laser_callback(self, msg):
        """Process laser scan data."""
        self.latest_laser = msg

    def odom_callback(self, msg):
        """Process odometry data."""
        self.latest_odom = msg

    def process_perception(self):
        """Process sensor data to extract meaningful information."""
        if self.latest_laser is None or self.latest_odom is None:
            return

        # Extract features from sensor data
        obstacles = self.extract_obstacles(self.latest_laser)
        position = self.extract_position(self.latest_odom)
        velocity = self.extract_velocity(self.latest_odom)

        # Create perception message
        perception_msg = {
            'timestamp': time.time(),
            'position': position,
            'velocity': velocity,
            'obstacles': obstacles,
            'safe_directions': self.calculate_safe_directions(obstacles)
        }

        # Publish perception data
        perception_str = str(perception_msg)
        msg = String()
        msg.data = perception_str
        self.perception_publisher.publish(msg)

        # Store in buffer for history
        self.perception_buffer.append(perception_msg)

    def extract_obstacles(self, laser_msg):
        """Extract obstacle information from laser scan."""
        obstacles = []
        safety_distance = 0.8  # meters

        for i, distance in enumerate(laser_msg.ranges):
            if 0 < distance < safety_distance:
                angle = laser_msg.angle_min + i * laser_msg.angle_increment
                obstacles.append({
                    'angle': angle,
                    'distance': distance,
                    'index': i
                })

        return obstacles

    def extract_position(self, odom_msg):
        """Extract position from odometry."""
        pos = odom_msg.pose.pose.position
        return {'x': pos.x, 'y': pos.y, 'z': pos.z}

    def extract_velocity(self, odom_msg):
        """Extract velocity from odometry."""
        vel = odom_msg.twist.twist
        return {'linear': {'x': vel.linear.x, 'y': vel.linear.y, 'z': vel.linear.z},
                'angular': {'x': vel.angular.x, 'y': vel.angular.y, 'z': vel.angular.z}}

    def calculate_safe_directions(self, obstacles):
        """Calculate safe movement directions based on obstacles."""
        if not obstacles:
            return ['forward', 'left', 'right', 'backward']

        # Simplified approach: divide space into sectors
        sectors = {'front': [], 'left': [], 'right': [], 'back': []}

        for obs in obstacles:
            angle_deg = np.degrees(obs['angle'])
            if -45 <= angle_deg < 45:
                sectors['front'].append(obs)
            elif 45 <= angle_deg < 135:
                sectors['left'].append(obs)
            elif -135 <= angle_deg < -45:
                sectors['right'].append(obs)
            else:
                sectors['back'].append(obs)

        safe_dirs = []
        if not sectors['front']:
            safe_dirs.append('forward')
        if not sectors['left']:
            safe_dirs.append('left')
        if not sectors['right']:
            safe_dirs.append('right')
        if not sectors['back']:
            safe_dirs.append('backward')

        return safe_dirs


class AIDecisionNode(Node):
    """Handles AI decision making based on perception data."""

    def __init__(self):
        super().__init__('ai_decision_node')

        # Subscriptions
        self.perception_subscription = self.create_subscription(
            String, 'perception_data', self.perception_callback, 10)

        # Publisher for AI decisions
        self.decision_publisher = self.create_publisher(
            String, 'ai_decision', 10)

        # Internal state
        self.current_perception = None
        self.decision_history = deque(maxlen=20)
        self.current_goal = {'x': 5.0, 'y': 5.0}  # Example goal position

        # Decision timer
        self.decision_timer = self.create_timer(0.05, self.make_decision)

    def perception_callback(self, msg):
        """Receive perception data from perception node."""
        try:
            # Parse perception data (in a real system, use proper message types)
            import ast
            self.current_perception = ast.literal_eval(msg.data)
        except Exception as e:
            self.get_logger().error(f'Error parsing perception data: {e}')

    def make_decision(self):
        """Make AI decisions based on current perception."""
        if self.current_perception is None:
            return

        # Example decision logic: navigate to goal while avoiding obstacles
        decision = self.navigate_with_obstacle_avoidance()

        # Publish decision
        decision_msg = String()
        decision_msg.data = decision
        self.decision_publisher.publish(decision_msg)

        # Store in history
        self.decision_history.append({
            'timestamp': self.current_perception['timestamp'],
            'decision': decision,
            'position': self.current_perception['position']
        })

    def navigate_with_obstacle_avoidance(self):
        """Navigate toward goal while avoiding obstacles."""
        if not self.current_perception:
            return 'stop'

        # Calculate direction to goal
        pos = self.current_perception['position']
        goal = self.current_goal

        dx = goal['x'] - pos['x']
        dy = goal['y'] - pos['y']
        goal_angle = np.arctan2(dy, dx)

        # Check if obstacles block the path to goal
        safe_directions = self.current_perception['safe_directions']

        # If path to goal is blocked, choose alternative
        if 'forward' not in safe_directions:
            # Simple obstacle avoidance: try left or right
            if 'left' in safe_directions:
                return 'turn_left'
            elif 'right' in safe_directions:
                return 'turn_right'
            else:
                return 'turn_around'

        # If path is clear, move toward goal
        if abs(goal_angle) < 0.2:  # Close to goal direction
            return 'forward'
        elif goal_angle > 0:
            return 'turn_left'
        else:
            return 'turn_right'


class AIActionNode(Node):
    """Executes actions based on AI decisions."""

    def __init__(self):
        super().__init__('ai_action_node')

        # Subscriptions
        self.decision_subscription = self.create_subscription(
            String, 'ai_decision', self.decision_callback, 10)

        # Publishers
        self.cmd_publisher = self.create_publisher(
            Twist, 'cmd_vel', 10)

        # Internal state
        self.current_decision = 'stop'
        self.last_decision_time = time.time()

        # Action timer
        self.action_timer = self.create_timer(0.02, self.execute_action)

    def decision_callback(self, msg):
        """Receive AI decision."""
        self.current_decision = msg.data
        self.last_decision_time = time.time()

    def execute_action(self):
        """Execute the current AI decision."""
        cmd = Twist()

        if self.current_decision == 'forward':
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
        elif self.current_decision == 'turn_left':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.4
        elif self.current_decision == 'turn_right':
            cmd.linear.x = 0.0
            cmd.angular.z = -0.4
        elif self.current_decision == 'turn_around':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.6
        elif self.current_decision == 'backward':
            cmd.linear.x = -0.2
            cmd.angular.z = 0.0
        else:  # stop or unknown decision
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Publish command
        self.cmd_publisher.publish(cmd)

        # Log the action
        self.get_logger().debug(f'Executing decision: {self.current_decision}')


class CompleteAIInterface(Node):
    """
    Complete AI-robot interface that integrates perception, decision making, and action.
    This is a combined node for demonstration purposes.
    """

    def __init__(self):
        super().__init__('complete_ai_interface')

        # All the functionality from the separate nodes
        self.laser_subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Internal state
        self.latest_laser = None
        self.latest_odom = None
        self.current_goal = {'x': 5.0, 'y': 5.0}
        self.safety_distance = 0.6

        # Processing timer
        self.process_timer = self.create_timer(0.05, self.ai_loop)

    def laser_callback(self, msg):
        self.latest_laser = msg

    def odom_callback(self, msg):
        self.latest_odom = msg

    def ai_loop(self):
        """Complete AI loop: perception -> decision -> action."""
        if self.latest_laser is None or self.latest_odom is None:
            return

        # 1. Perception: Process sensor data
        obstacles = self.process_laser_data(self.latest_laser)
        position = self.get_position(self.latest_odom)

        # 2. Decision: Make intelligent decision
        decision = self.make_navigation_decision(obstacles, position)

        # 3. Action: Execute decision
        cmd = self.decide_command(decision)
        self.cmd_publisher.publish(cmd)

    def process_laser_data(self, laser_msg):
        """Process laser data to detect obstacles."""
        obstacles = []
        for i, distance in enumerate(laser_msg.ranges):
            if 0 < distance < self.safety_distance:
                angle = laser_msg.angle_min + i * laser_msg.angle_increment
                obstacles.append({'angle': angle, 'distance': distance})
        return obstacles

    def get_position(self, odom_msg):
        """Extract position from odometry."""
        pos = odom_msg.pose.pose.position
        return {'x': pos.x, 'y': pos.y}

    def make_navigation_decision(self, obstacles, position):
        """Make navigation decision based on obstacles and goal."""
        if obstacles:
            # If obstacles detected, avoid them
            return "avoid_obstacle"
        else:
            # Navigate toward goal
            goal = self.current_goal
            dx = goal['x'] - position['x']
            dy = goal['y'] - position['y']
            angle_to_goal = np.arctan2(dy, dx)

            if abs(angle_to_goal) < 0.2:
                return "forward"
            elif angle_to_goal > 0:
                return "turn_left"
            else:
                return "turn_right"

    def decide_command(self, decision):
        """Convert decision to Twist command."""
        cmd = Twist()
        if decision == "forward":
            cmd.linear.x = 0.3
        elif decision == "turn_left":
            cmd.angular.z = 0.4
        elif decision == "turn_right":
            cmd.angular.z = -0.4
        elif decision == "avoid_obstacle":
            cmd.angular.z = 0.5  # Turn to avoid
        return cmd


def main_perception(args=None):
    """Run the perception node."""
    rclpy.init(args=args)
    node = AIPerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_decision(args=None):
    """Run the decision node."""
    rclpy.init(args=args)
    node = AIDecisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_action(args=None):
    """Run the action node."""
    rclpy.init(args=args)
    node = AIActionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_complete(args=None):
    """Run the complete AI interface."""
    rclpy.init(args=args)
    node = CompleteAIInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import sys

    print("AI Robot Interface Examples")
    print("Run with 'python3 ai_robot_interface.py perception' for perception node")
    print("Run with 'python3 ai_robot_interface.py decision' for decision node")
    print("Run with 'python3 ai_robot_interface.py action' for action node")
    print("Run with 'python3 ai_robot_interface.py complete' for complete interface")

    if len(sys.argv) > 1:
        if sys.argv[1] == 'perception':
            main_perception()
        elif sys.argv[1] == 'decision':
            main_decision()
        elif sys.argv[1] == 'action':
            main_action()
        elif sys.argv[1] == 'complete':
            main_complete()
    else:
        main_complete()  # Default to complete example