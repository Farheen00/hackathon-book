---
sidebar_position: 2
title: Depth Camera Simulation
description: Implementing depth camera simulation with realistic depth maps
id: ch2-s8-depth-camera-simulation
---

# Depth Camera Simulation

This section covers implementing depth camera simulation with realistic depth maps. You'll learn how to configure depth cameras that generate realistic depth data matching physical properties.

## Key Concepts

- Depth camera principles
- Depth map generation
- Camera parameters and configuration
- ROS 2 message formats for depth data

## Diagram Descriptions

1. **Depth Camera Operation**: A visual representation showing how depth cameras measure distances to create depth maps.
2. **Depth Map Structure**: A diagram illustrating the structure of depth map data and how pixel values represent distances.

## Content

Depth cameras provide 3D spatial information by measuring the distance to objects in the scene. In simulation, depth cameras generate realistic depth maps that match real-world sensor patterns and physical properties.

### Depth Camera Parameters

Key depth camera parameters include:

- Resolution: Image width and height in pixels
- Field of view: Horizontal and vertical viewing angles
- Near and far clipping: Minimum and maximum measurable distances
- Noise models: Simulation of real sensor noise characteristics

### ROS 2 Integration

Depth camera data in ROS 2 follows standard message formats:

- sensor_msgs/Image: For depth maps with 32-bit float encoding
- sensor_msgs/CameraInfo: For camera intrinsic parameters

## Example

Here's how to configure a depth camera sensor in Gazebo:

```xml
<sensor name="depth_camera" type="depth_camera">
  <pose>0.0 0.0 0.5 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::DepthCamera">
    <topic>depth_camera/image</topic>
  </plugin>
</sensor>
```

To verify the depth camera output:

```bash
# Check depth camera data
ros2 topic echo /depth_camera/image sensor_msgs/msg/Image
```