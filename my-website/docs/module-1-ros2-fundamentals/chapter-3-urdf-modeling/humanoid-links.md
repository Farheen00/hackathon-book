---
sidebar_position: 3
---

# Humanoid Robot Links and Joints

## Humanoid Robot Anatomy

Humanoid robots are designed to mimic human form and locomotion. A typical humanoid robot consists of:

- **Torso**: The central body containing the core systems
- **Head**: Contains sensors (cameras, microphones) and sometimes displays
- **Arms**: Two arms with shoulders, elbows, wrists, and hands
- **Legs**: Two legs with hips, knees, ankles, and feet
- **Joints**: Multiple degrees of freedom to enable human-like movement

## Basic Humanoid URDF Structure

A humanoid robot URDF typically follows this kinematic structure:

```
base_link (usually torso)
├── head
├── left_arm
│   ├── left_shoulder
│   ├── left_elbow
│   └── left_wrist
├── right_arm
│   ├── right_shoulder
│   ├── right_elbow
│   └── right_wrist
├── left_leg
│   ├── left_hip
│   ├── left_knee
│   └── left_ankle
└── right_leg
    ├── right_hip
    ├── right_knee
    └── right_ankle
```

## Complete Humanoid URDF Example

Here's a detailed URDF for a simple humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Torso Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <mass value="5.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.8 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.004"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="2"/>
  </joint>

  <!-- Left Arm -->
  <!-- Left Shoulder -->
  <link name="left_shoulder">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="joint_color">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder"/>
    <origin xyz="0.15 0.1 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
  </joint>

  <!-- Left Upper Arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.6 0.6 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="1.57 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.36" upper="0" effort="15" velocity="2"/>
  </joint>

  <!-- Left Forearm -->
  <link name="left_forearm">
    <visual>
      <origin xyz="0 0 -0.12" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.24"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.6 0.6 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.12" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.24"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.12" rpy="1.57 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="left_forearm_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2"/>
  </joint>

  <!-- Right Arm (symmetric to left arm) -->
  <link name="right_shoulder">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="joint_color">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_shoulder"/>
    <origin xyz="0.15 -0.1 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.6 0.6 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="1.57 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.36" upper="0" effort="15" velocity="2"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <origin xyz="0 0 -0.12" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.24"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.6 0.6 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.12" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.24"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.12" rpy="1.57 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="right_forearm_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_hip">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.1"/>
      </geometry>
      <material name="joint_color">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="-0.15 0.08 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="30" velocity="2"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.4 0.4 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_hip"/>
    <child link="left_thigh"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="2.36" effort="30" velocity="2"/>
  </joint>

  <link name="left_shin">
    <visual>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.4 0.4 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.0015"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="2"/>
  </joint>

  <!-- Right Leg (symmetric to left leg) -->
  <link name="right_hip">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.1"/>
      </geometry>
      <material name="joint_color">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_hip"/>
    <origin xyz="-0.15 -0.08 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="30" velocity="2"/>
  </joint>

  <link name="right_thigh">
    <visual>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.4 0.4 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_hip"/>
    <child link="right_thigh"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="2.36" effort="30" velocity="2"/>
  </joint>

  <link name="right_shin">
    <visual>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.4 0.4 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.0015"/>
    </inertial>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="2"/>
  </joint>

  <!-- Foot links would be added here for complete model -->
</robot>
```

## Humanoid Joint Considerations

### Degrees of Freedom (DOF)
Humanoid robots typically have 20-40+ DOF depending on complexity:

- **Head**: 3 DOF (pitch, yaw, roll)
- **Arms**: 6-7 DOF each (shoulder: 3, elbow: 1, wrist: 2-3)
- **Legs**: 6 DOF each (hip: 3, knee: 1, ankle: 2)

### Joint Limit Recommendations
- **Shoulders**: ±90° to ±120° for rotation
- **Elbows**: 0° to -150° (flexion only)
- **Wrists**: ±90° for rotation and flexion
- **Hips**: ±30° for rotation, 0° to 120° for flexion
- **Knees**: 0° to 120° (extension only)
- **Ankles**: ±30° for rotation and flexion

## Common Humanoid URDF Patterns

### 1. Fixed Base vs Floating Base
```xml
<!-- For simulation with fixed base -->
<joint name="world_joint" type="fixed">
  <parent link="world"/>
  <child link="base_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<!-- For floating base (full mobility) -->
<!-- Omit the world_joint or use a floating joint type -->
```

### 2. Transmission Elements for Control
```xml
<!-- Example transmission for a joint -->
<transmission name="left_elbow_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_elbow_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_elbow_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### 3. Gazebo-Specific Elements
```xml
<!-- Add Gazebo-specific properties -->
<gazebo reference="base_link">
  <material>Gazebo/Gray</material>
</gazebo>

<!-- Gazebo plugin for joint control -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/simple_humanoid</robotNamespace>
  </plugin>
</gazebo>
```

## Humanoid URDF Validation

### Checking Kinematic Chains
```bash
# Verify the URDF structure
check_urdf simple_humanoid.urdf

# Generate a graphical representation
urdf_to_graphiz simple_humanoid.urdf
```

### Visualization in RViz
```xml
<!-- To visualize in RViz, you'll need a robot_state_publisher node -->
<!-- Example launch command -->
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat simple_humanoid.urdf)
```

## Advanced Humanoid Features

### 1. Mimic Joints
For symmetric movements:
```xml
<joint name="right_elbow_joint" type="revolute">
  <parent link="right_shoulder"/>
  <child link="right_upper_arm"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-2.36" upper="0" effort="15" velocity="2"/>
  <mimic joint="left_elbow_joint" multiplier="1" offset="0"/>
</joint>
```

### 2. Safety Controllers
```xml
<!-- Joint limits with safety margins -->
<limit lower="${-M_PI/2*0.9}" upper="${M_PI/2*0.9}" effort="30" velocity="2"/>
```

## Best Practices for Humanoid URDF

### 1. Proportional Design
- Use realistic human proportions (head ~1/8 of total height)
- Ensure limbs are appropriately sized relative to torso
- Consider the robot's intended function in proportions

### 2. Center of Mass
- Position the torso link's origin near the geometric center
- Ensure overall center of mass is within the support polygon for stability
- Use realistic mass distributions

### 3. Joint Configuration
- Start with neutral positions (arms at sides, legs straight)
- Ensure joint limits prevent self-collision
- Consider the robot's workspace requirements

## Summary

Humanoid URDF modeling requires careful attention to anatomical structure, kinematic chains, and physical properties. The examples provided demonstrate proper link and joint definitions for a complete humanoid robot. Remember to validate your URDF files and consider the specific requirements of your application, whether for simulation, visualization, or control.

In the next section, we'll cover URDF validation techniques and best practices for ensuring your humanoid robot models are correct and ready for use in ROS applications.