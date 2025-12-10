---
sidebar_position: 2
---

# URDF Structure and Components

## What is URDF?

**URDF (Unified Robot Description Format)** is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links, joints, and their kinematic relationships. URDF is fundamental for robot simulation, visualization, and motion planning.

## Basic URDF Structure

A URDF file is an XML document with a specific structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links definition -->
  <link name="link_name">
    <!-- Visual properties -->
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>

    <!-- Collision properties -->
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>

    <!-- Inertial properties -->
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints definition -->
  <joint name="joint_name" type="revolute">
    <parent link="parent_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Links

**Links** represent rigid parts of the robot. Each link can have:

- **Visual**: How the link appears in visualizations
- **Collision**: How the link interacts in collision detection
- **Inertial**: Physical properties for simulation

### Link Components

#### Visual Properties
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Different geometry types -->
    <box size="0.1 0.1 0.1"/>
    <!-- OR -->
    <cylinder radius="0.05" length="0.1"/>
    <!-- OR -->
    <sphere radius="0.05"/>
    <!-- OR -->
    <mesh filename="package://my_robot/meshes/link.stl"/>
  </geometry>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
</visual>
```

#### Collision Properties
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Usually simpler than visual geometry for performance -->
    <box size="0.1 0.1 0.1"/>
  </geometry>
</collision>
```

#### Inertial Properties
```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
</inertial>
```

## Joints

**Joints** define the connections between links and their movement capabilities. URDF supports several joint types:

### Joint Types

1. **revolute**: Rotational joint with limited range
2. **continuous**: Rotational joint without limits
3. **prismatic**: Linear sliding joint with limits
4. **fixed**: No movement (rigid connection)
5. **floating**: 6 DOF movement (rarely used)
6. **planar**: Movement in a plane (rarely used)

### Joint Structure
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Joint Components Explained

- **parent/child**: Links that the joint connects
- **origin**: Position and orientation of the joint relative to parent
- **axis**: Axis of rotation or translation (for revolute/prismatic joints)
- **limit**: Movement constraints (not needed for continuous/fixed joints)
- **dynamics**: Physical properties like damping and friction

## Complete Simple URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base and arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

## URDF Best Practices

### 1. Naming Conventions
- Use descriptive, consistent names
- Use underscores for multi-word names: `left_arm_joint`
- Follow a consistent hierarchy: `base_link`, `torso`, `head`, etc.

### 2. Coordinate Frames
- Follow the ROS coordinate frame conventions:
  - X: forward
  - Y: left
  - Z: up
- Use consistent orientations across the robot

### 3. Inertial Properties
- Always include inertial properties for simulation
- Use realistic values or estimate based on geometry
- Ensure inertia matrix is physically valid (positive definite)

### 4. Geometry Optimization
- Use simple collision geometries for performance
- Use detailed mesh geometries for visualization
- Consider using multiple collision elements for complex shapes

### 5. Joint Limits
- Always specify joint limits for revolute joints
- Use realistic limits based on physical constraints
- Consider safety margins in limit selection

## URDF Validation

### Syntax Validation
```bash
# Check if URDF is syntactically correct
check_urdf /path/to/your/robot.urdf
```

### Visualization
```bash
# Visualize URDF in RViz
ros2 run rviz2 rviz2
# Then add RobotModel display and specify the URDF file
```

### Graph Generation
```bash
# Generate a visual graph of the URDF structure
urdf_to_graphiz /path/to/your/robot.urdf
```

## Common URDF Elements

### Materials
```xml
<material name="red">
  <color rgba="1 0 0 1"/>
</material>
<material name="blue">
  <color rgba="0 0 1 1"/>
</material>
```

### Transmission Elements (for actuator control)
```xml
<transmission name="joint1_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint1">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="joint1_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## URDF Tools and Utilities

### Essential ROS Tools
- `check_urdf`: Validate URDF syntax
- `urdf_to_graphiz`: Generate structure graph
- `joint_state_publisher`: Publish joint states for visualization
- `robot_state_publisher`: Publish TF transforms from URDF and joint states

### Visualization Tools
- RViz: 3D visualization of robot models
- Gazebo: Physics simulation with URDF models
- MeshLab: For working with mesh files

## Summary

URDF provides the fundamental structure for describing robots in ROS. Understanding its components (links, joints, visual, collision, and inertial properties) is essential for creating robot models that work correctly in simulation, visualization, and control applications. The next section will focus on how to apply these concepts specifically to humanoid robot modeling.