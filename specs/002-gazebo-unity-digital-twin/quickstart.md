# Quickstart Guide: Module 2 – The Digital Twin (Gazebo & Unity)

## Prerequisites

Before starting this module, ensure you have:

1. **Gazebo Installation**:
   - Gazebo Garden or Harmonic installed
   - ROS 2 Humble Hawksbill or Iron Irwini
   - Basic command-line familiarity

2. **Unity Installation**:
   - Unity 2022.3 LTS installed
   - Unity Hub for project management
   - Basic understanding of Unity interface

3. **Development Environment**:
   - Text editor or IDE with Python support
   - Terminal/shell access
   - Git for version control (optional but recommended)

4. **Basic Knowledge**:
   - Completion of Module 1 (ROS 2 fundamentals)
   - Understanding of basic physics concepts
   - Familiarity with 3D coordinate systems

## Setting Up Your Environment

### 1. Gazebo Environment Setup

```bash
# Source your ROS 2 installation
source /opt/ros/humble/setup.bash  # For ROS 2 Humble
# OR
source /opt/ros/iron/setup.bash    # For ROS 2 Iron

# Verify Gazebo installation
gz version
gz sim --version

# Create a workspace for examples
mkdir -p ~/gazebo_unity_ws/src
cd ~/gazebo_unity_ws
colcon build
source install/setup.bash
```

### 2. Verify Installation

```bash
# Check Gazebo can launch
gz sim

# Check ROS 2-Gazebo bridge
ros2 pkg list | grep gazebo
```

### 3. Unity Setup

1. Launch Unity Hub
2. Install Unity 2022.3 LTS
3. Create a new 3D project named "DigitalTwinExamples"
4. Install ROS-TCP-Connector package from Package Manager

## Running Your First Gazebo Example

### 1. Simple Physics World

Create a basic world file:

```xml
<!-- simple_physics.sdf -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_physics">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>

    <model name="ground_plane">
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

    <model name="falling_box">
      <pose>0 0 2 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### 2. Running the Example

```bash
# Launch Gazebo with your world
gz sim simple_physics.sdf

# Or run with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py world_file:=path/to/simple_physics.sdf
```

## Running Your First Unity Example

### 1. Basic Scene Setup

1. In Unity, create a new 3D scene
2. Add a plane as the ground
3. Add a cube above the plane
4. Add basic lighting (Directional Light)

### 2. Adding Physics

1. Add Rigidbody component to the cube
2. Adjust mass and drag properties
3. Play the scene to see physics simulation

## Sensor Simulation Examples

### 1. LiDAR Simulation

Configure a LiDAR sensor in Gazebo:

```xml
<sensor name="lidar" type="ray">
  <pose>0.0 0.0 0.1 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle>
        <max_angle>1.570796</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensor">
    <topic>lidar_scan</topic>
  </plugin>
</sensor>
```

### 2. Verify Sensor Output

```bash
# Check LiDAR data
ros2 topic echo /lidar_scan sensor_msgs/msg/LaserScan
```

## Unity-ROS Integration

### 1. Setting up ROS-TCP-Connector

1. In Unity, add ROSConnection object to scene
2. Configure IP address and port to connect to ROS 2 bridge
3. Create publisher/subscriber scripts for data exchange

### 2. Basic Integration Script

```csharp
using ROS2;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class SensorPublisher : MonoBehaviour
{
    ROSConnection ros;
    string topicName = "/unity_sensor_data";

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.instance;
    }

    void FixedUpdate()
    {
        // Publish sensor data to ROS
        var sensorData = new Float32Msg();
        sensorData.data = transform.position.y;
        ros.Publish<Float32Msg>(topicName, sensorData);
    }
}
```

## Testing Integration

### 1. Combined Gazebo-Unity Simulation

1. Launch Gazebo simulation with ROS bridge
2. Launch Unity scene with ROS-TCP-Connector
3. Verify data exchange between systems

```bash
# Terminal 1: Launch Gazebo with ROS bridge
ros2 launch gazebo_ros empty_world.launch.py

# Terminal 2: Launch Unity scene
# (Run Unity application with ROS integration)

# Terminal 3: Monitor topics
ros2 topic list
ros2 topic echo /sensor_data
```

## Troubleshooting Common Issues

### Gazebo Environment Not Found
- Ensure you've sourced the correct ROS 2 setup file
- Check that Gazebo is properly installed: `gz version`

### Unity-ROS Connection Issues
- Verify IP addresses match between Unity and ROS systems
- Check firewall settings that might block connections
- Ensure ROS-TCP-Connector is properly configured

### Physics Simulation Problems
- Check that models have proper mass and inertia values
- Verify collision and visual elements are defined
- Adjust physics engine parameters if simulation is unstable

## Next Steps

1. Complete Chapter 1: Gazebo Physics Essentials
2. Practice with the provided simulation examples
3. Move to Chapter 2: Unity for Humanoid Interaction
4. Explore Chapter 3: Simulated Sensors