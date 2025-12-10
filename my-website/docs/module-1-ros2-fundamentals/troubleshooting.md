---
sidebar_position: 102
---

# Troubleshooting Guide - Module 1

## Overview

This troubleshooting guide addresses common issues encountered when working with ROS 2 fundamentals, Python-ROS integration, and URDF modeling. Use this guide to diagnose and resolve problems in your robotic systems.

## Common ROS 2 Issues

### 1. Nodes Not Communicating

**Symptoms**:
- Publishers not reaching subscribers
- Services not responding
- `ros2 topic list` shows no topics

**Causes and Solutions**:
- **Different ROS_DOMAIN_ID**: Ensure all nodes are using the same domain ID
  ```bash
  # Check domain ID
  echo $ROS_DOMAIN_ID
  # Set domain ID if needed
  export ROS_DOMAIN_ID=0
  ```

- **Node not properly initialized**: Check that `rclpy.init()` is called
  ```python
  # Correct initialization
  rclpy.init(args=args)
  node = MyNode()
  rclpy.spin(node)
  ```

- **Topic name mismatch**: Verify topic names match exactly (including case)
  ```python
  # Publisher and subscriber must use identical topic names
  publisher = self.create_publisher(MsgType, 'topic_name', 10)
  subscription = self.create_subscription(MsgType, 'topic_name', callback, 10)
  ```

- **Message type mismatch**: Ensure publisher and subscriber use the same message type
  ```python
  # Both must use the same message type
  from std_msgs.msg import String  # Use same type in both files
  ```

### 2. Import Errors

**Symptoms**:
- `ModuleNotFoundError: No module named 'rclpy'`
- `ImportError: No module named 'std_msgs.msg'`

**Solutions**:
- **ROS 2 not sourced**: Source your ROS 2 installation
  ```bash
  source /opt/ros/humble/setup.bash  # For Humble
  # OR
  source /opt/ros/iron/setup.bash    # For Iron
  ```

- **Python path issues**: Run Python with ROS 2 environment
  ```bash
  python3 -c "import rclpy; print('rclpy imported successfully')"
  ```

### 3. Permission Issues

**Symptoms**:
- "Permission denied" errors when running nodes
- Cannot create log files

**Solutions**:
- Ensure proper file permissions
- Don't run ROS 2 nodes with `sudo` (security risk)
- Check that ROS 2 installation has proper permissions

## Common Python-ROS Integration Issues

### 1. Callback Execution Problems

**Symptoms**:
- Callbacks not executing
- Delayed message processing
- Node freezing

**Causes and Solutions**:
- **Not using rclpy.spin()**: Ensure you're spinning the node
  ```python
  # Correct usage
  rclpy.init(args=args)
  node = MyNode()
  rclpy.spin(node)  # This keeps the node running
  ```

- **Blocking operations in callbacks**: Avoid long operations in callbacks
  ```python
  # Bad: Blocking operation in callback
  def callback(self, msg):
      time.sleep(5)  # This will block other callbacks

  # Good: Use separate threads or async for long operations
  def callback(self, msg):
      # Schedule long operation in separate thread
      thread = threading.Thread(target=self.long_operation, args=(msg,))
      thread.start()
  ```

### 2. Memory Issues in Long-Running Systems

**Symptoms**:
- Memory usage increases over time
- System slows down after extended operation

**Solutions**:
- **Use bounded buffers**: Limit queue sizes
  ```python
  # Use appropriate queue depths
  self.publisher = self.create_publisher(MsgType, 'topic', 10)
  self.subscription = self.create_subscription(MsgType, 'topic', callback, 10)
  ```

- **Clear data structures periodically**: Use `collections.deque` with max size
  ```python
  from collections import deque

  # Bounded data structure
  self.data_buffer = deque(maxlen=100)
  ```

### 3. Threading and Concurrency Issues

**Symptoms**:
- Race conditions
- Inconsistent behavior
- Node crashes

**Solutions**:
- **Use appropriate executor**: For multi-threaded nodes
  ```python
  from rclpy.executors import MultiThreadedExecutor

  executor = MultiThreadedExecutor()
  executor.add_node(node)
  executor.spin()
  ```

- **Protect shared data**: Use locks for shared state
  ```python
  import threading

  self.lock = threading.Lock()
  self.shared_data = {}

  def callback(self, msg):
      with self.lock:
          self.shared_data['key'] = msg.data
  ```

## Common URDF Issues

### 1. URDF Validation Errors

**Symptoms**:
- `check_urdf` reports errors
- Robot model doesn't load in RViz
- Simulation fails

**Common Errors and Solutions**:

- **Missing inertial properties**:
  ```xml
  <!-- Always include inertial properties -->
  <link name="my_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  ```

- **Duplicate link names**:
  ```xml
  <!-- Bad: duplicate names -->
  <link name="arm"/>
  <link name="arm"/>  <!-- Error! -->

  <!-- Good: unique names -->
  <link name="left_arm"/>
  <link name="right_arm"/>
  ```

- **Invalid joint references**:
  ```xml
  <!-- Bad: references non-existent links -->
  <joint name="joint1" type="revolute">
    <parent link="nonexistent_link"/>  <!-- Error! -->
    <child link="another_link"/>
  </joint>
  ```

### 2. Inertia Matrix Issues

**Symptoms**:
- Robot falls through ground in simulation
- Unstable physics behavior
- "Inertia matrix is not positive definite" errors

**Solutions**:
- **Ensure positive diagonal values**:
  ```xml
  <!-- Good: positive diagonal values -->
  <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.2"/>
  ```

- **Satisfy triangle inequality** (ixx + iyy ≥ izz, etc.):
  ```xml
  <!-- Good: satisfies triangle inequality -->
  <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.15"/>
  ```

### 3. Visualization Problems

**Symptoms**:
- Robot appears invisible in RViz
- Wrong colors or shapes
- Missing parts

**Solutions**:
- **Check visual geometry**:
  ```xml
  <visual>
    <geometry>
      <box size="0.1 0.1 0.1"/>  <!-- Ensure geometry is defined -->
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>  <!-- Ensure material is defined -->
    </material>
  </visual>
  ```

- **Verify file paths for meshes**:
  ```xml
  <!-- Use proper package:// format -->
  <mesh filename="package://my_robot/meshes/link.stl"/>
  ```

## Debugging Strategies

### 1. System Introspection

Use ROS 2 command-line tools to understand system state:

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /topic_name std_msgs/msg/String

# Get information about a node
ros2 node info node_name

# Get information about a topic
ros2 topic info /topic_name
```

### 2. Logging and Debugging

Add logging to your nodes for debugging:

```python
def my_function(self):
    self.get_logger().info('Function called with parameters')
    self.get_logger().debug('Detailed debug information')
    self.get_logger().error('Error occurred')
```

### 3. Using rqt Tools

Visual debugging tools:

```bash
# Graph of all nodes and topics
rqt_graph

# Topic monitor
rqt_topic

# Service caller
rqt_service_caller
```

## Performance Optimization

### 1. Message Rate Issues

**Problem**: Too many messages causing performance issues

**Solutions**:
- **Throttle message publication**:
  ```python
  # Use timers to control rate
  self.timer = self.create_timer(0.1, self.publish_callback)  # 10 Hz
  ```

- **Use appropriate QoS settings**:
  ```python
  from rclpy.qos import QoSProfile, QoSHistoryPolicy

  qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST)
  self.publisher = self.create_publisher(MsgType, 'topic', qos)
  ```

### 2. CPU Usage Optimization

**Problem**: High CPU usage from ROS nodes

**Solutions**:
- **Optimize callback frequency**:
  ```python
  # Don't process every message if not needed
  self.message_counter = 0
  def callback(self, msg):
      self.message_counter += 1
      if self.message_counter % 5 == 0:  # Process every 5th message
          self.process_message(msg)
  ```

- **Use efficient data structures**:
  ```python
  # Use numpy for numerical computations
  import numpy as np
  data = np.array(sensor_data)  # More efficient than Python lists for math
  ```

## Safety Considerations

### 1. Emergency Stop Implementation

Always implement emergency stop functionality:

```python
class SafeRobotController(Node):
    def __init__(self):
        super().__init__('safe_controller')
        self.emergency_stop = False
        self.emergency_sub = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_callback, 1)

    def emergency_callback(self, msg):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.stop_robot()

    def stop_robot(self):
        # Send stop command to robot
        stop_cmd = Twist()
        self.cmd_publisher.publish(stop_cmd)
```

### 2. Input Validation

Always validate inputs from sensors or external sources:

```python
def sensor_callback(self, msg):
    # Validate sensor data
    if any(np.isnan(msg.ranges) or r < 0 for r in msg.ranges):
        self.get_logger().warn('Invalid sensor data received')
        return

    # Process valid data
    self.process_valid_sensor_data(msg)
```

## Quick Reference

### Common Commands
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Check URDF
check_urdf robot.urdf

# List ROS 2 components
ros2 node list && ros2 topic list && ros2 service list

# Echo topic with specific type
ros2 topic echo /scan sensor_msgs/msg/LaserScan
```

### Common Error Messages and Solutions
- `"Failed to initialize rcl" ` → ROS 2 not sourced or node name conflict
- `"Topic does not exist"` → Check topic names and timing of node startup
- `"Inertia matrix is not positive definite"` → Fix inertia values in URDF
- `"Permission denied"` → Check file permissions, don't use sudo

## Getting Help

When seeking help with ROS 2 issues:

1. **Provide detailed error messages** including full traceback
2. **Include your code** (minimal example that reproduces the issue)
3. **Specify your environment** (ROS 2 version, OS, Python version)
4. **Show what you've tried** to solve the problem

Useful resources:
- ROS Answers: https://answers.ros.org/
- ROS Discourse: https://discourse.ros.org/
- GitHub issues for specific packages
- Local ROS community groups

Remember: most ROS 2 issues have standard solutions. Check the official documentation and community resources before posting new questions.