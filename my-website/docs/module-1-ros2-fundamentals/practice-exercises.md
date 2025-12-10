---
sidebar_position: 101
---

# Practice Exercises - Module 1

## Overview

This page contains hands-on exercises to reinforce the concepts learned in Module 1. Each exercise builds on the previous ones, helping you gain practical experience with ROS 2 fundamentals, Python-ROS integration, and URDF modeling.

## Chapter 1 Exercises: ROS 2 Fundamentals

### Exercise 1.1: Basic Publisher-Subscriber
**Objective**: Create a publisher that publishes temperature readings and a subscriber that logs them.

**Steps**:
1. Create a publisher node that publishes `std_msgs/Float32` messages with simulated temperature data
2. Create a subscriber node that receives temperature data and logs it with timestamps
3. Test the communication between nodes
4. Verify that messages are received correctly

**Expected Outcome**: Temperature values published by one node are received and logged by the other.

### Exercise 1.2: Custom Service
**Objective**: Create a service that converts temperatures between Celsius and Fahrenheit.

**Steps**:
1. Define a custom service file that takes a temperature value and unit conversion request
2. Implement a service server that performs the conversion
3. Create a service client that sends requests to the server
4. Test the service with various temperature values

**Expected Outcome**: Service correctly converts temperatures between Celsius and Fahrenheit.

### Exercise 1.3: Multiple Topics
**Objective**: Create a node that subscribes to multiple topics and publishes a combined result.

**Steps**:
1. Create nodes that publish simulated sensor data (e.g., temperature, humidity, pressure)
2. Create a fusion node that subscribes to all sensor topics
3. Have the fusion node calculate and publish a derived value (e.g., heat index)
4. Test the system with all nodes running

**Expected Outcome**: Fusion node correctly combines data from multiple sensors.

## Chapter 2 Exercises: Python-ROS Integration

### Exercise 2.1: Simple AI Controller
**Objective**: Create a Python-based AI controller that navigates a robot to avoid obstacles.

**Steps**:
1. Create a subscriber to laser scan data (`sensor_msgs/LaserScan`)
2. Implement a simple obstacle avoidance algorithm in Python
3. Publish velocity commands to the robot (`geometry_msgs/Twist`)
4. Test the controller in simulation

**Expected Outcome**: Robot successfully navigates while avoiding obstacles.

### Exercise 2.2: State Machine Behavior
**Objective**: Implement a state machine that controls robot behavior based on sensor input.

**Steps**:
1. Define states (e.g., SEARCHING, APPROACHING_OBJECT, AVOIDING_OBSTACLE)
2. Create a state machine that transitions based on sensor data
3. Implement different behaviors for each state
4. Test the state transitions in simulation

**Expected Outcome**: Robot exhibits different behaviors based on its state and sensor input.

### Exercise 2.3: Learning Algorithm
**Objective**: Implement a simple learning algorithm that improves robot navigation over time.

**Steps**:
1. Create a system that tracks robot position and obstacles
2. Implement a basic Q-learning algorithm for path planning
3. Update the Q-table based on robot experiences
4. Test that the robot's navigation improves over time

**Expected Outcome**: Robot's navigation performance improves as it learns from experience.

## Chapter 3 Exercises: URDF Modeling

### Exercise 3.1: Simple Robot Arm
**Objective**: Create a URDF model of a simple 3-DOF robot arm.

**Steps**:
1. Define the base, two links, and end effector
2. Create appropriate joints (likely revolute joints)
3. Add visual and collision geometry
4. Validate the URDF file
5. Visualize the robot in RViz

**Expected Outcome**: A functional 3-DOF robot arm that can be visualized and simulated.

### Exercise 3.2: Mobile Robot
**Objective**: Create a URDF model of a differential drive mobile robot.

**Steps**:
1. Define the main body/chassis
2. Add two wheels with appropriate joints
3. Include caster wheel(s) for stability
4. Add sensors (e.g., camera, laser scanner) as appropriate
5. Validate and test the model

**Expected Outcome**: A mobile robot model that can be used for navigation simulation.

### Exercise 3.3: Humanoid Appendage
**Objective**: Create a detailed URDF model of a humanoid arm with hand.

**Steps**:
1. Model shoulder, upper arm, forearm, and hand
2. Include appropriate joint limits for human-like movement
3. Add realistic visual geometry
4. Specify proper inertial properties
5. Validate the complete model

**Expected Outcome**: A realistic humanoid arm model with proper kinematics.

## Comprehensive Integration Exercise

### Exercise C.1: Complete AI-Robot System
**Objective**: Integrate all concepts learned in the module into a complete system.

**System Requirements**:
1. A mobile robot with sensors (laser scanner, camera)
2. A URDF model of the robot
3. An AI controller that processes sensor data
4. Navigation behaviors for goal-seeking
5. Obstacle avoidance capabilities
6. Learning component for improved navigation

**Implementation Steps**:
1. Create the robot URDF model
2. Implement sensor data processing nodes
3. Create the AI decision-making node
4. Implement action execution nodes
5. Add learning components for adaptive behavior
6. Test the complete system in simulation

**Expected Outcome**: A complete system where an AI agent processes sensor data to navigate a robot safely to goals while avoiding obstacles and learning to improve performance.

## Troubleshooting Tips

### Common Issues and Solutions

1. **Nodes not communicating**:
   - Verify nodes are on the same ROS domain
   - Check topic names match exactly
   - Ensure message types are compatible

2. **URDF validation errors**:
   - Check for missing inertial properties
   - Verify all joint references are valid
   - Ensure mass values are positive

3. **Simulation instability**:
   - Check inertia values are physically plausible
   - Verify joint limits are reasonable
   - Ensure proper mass distribution

4. **Python-ROS bridge issues**:
   - Verify rclpy is properly initialized
   - Check callback functions are properly defined
   - Ensure proper message type conversions

## Self-Assessment Checklist

After completing these exercises, you should be able to:
- [ ] Create and run basic ROS 2 nodes in Python
- [ ] Implement publish-subscribe and service communication patterns
- [ ] Integrate AI algorithms with ROS 2 systems
- [ ] Create valid URDF models for robots
- [ ] Validate and test robot models
- [ ] Debug common ROS 2 issues
- [ ] Design safe and effective AI-robot interfaces

## Extension Activities

For additional challenge, consider:
1. Adding more complex sensors to your robot models
2. Implementing more sophisticated AI algorithms
3. Creating multi-robot systems
4. Adding real-time constraints to your systems
5. Implementing safety mechanisms for robot operation

## Resources for Further Learning

- ROS 2 Tutorials: http://docs.ros.org/en/rolling/Tutorials.html
- URDF Documentation: http://wiki.ros.org/urdf
- rclpy Documentation: http://docs.ros.org/en/rolling/p/rclpy/
- Gazebo Simulation: http://gazebosim.org/tutorials

Complete these exercises to solidify your understanding of ROS 2 fundamentals, AI-robot integration, and robot modeling. Each exercise builds on the previous ones, creating a comprehensive understanding of the concepts covered in Module 1.