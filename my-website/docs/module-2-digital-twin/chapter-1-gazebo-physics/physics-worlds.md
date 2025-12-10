---
sidebar_position: 1
title: Physics Worlds
description: Creating physics environments in Gazebo with proper gravity configuration
id: ch2-s1-physics-worlds
---

# Physics Worlds

This section covers creating physics environments in Gazebo with proper gravity configuration. You'll learn how to set up realistic physics parameters for humanoid robot simulation.

## Key Concepts

- Gravity configuration in Gazebo
- Physics engine parameters
- World file structure
- Collision properties

## Diagram Descriptions

1. **Gazebo World Structure**: A diagram showing the hierarchical structure of a Gazebo world file with physics, models, and plugins.
2. **Gravity Vector**: A visual representation of the gravity vector (0, 0, -9.8) in 3D space affecting objects in the simulation.

## Content

In Gazebo, physics worlds are defined using SDF (Simulation Description Format) files. These files contain all the necessary information to simulate a 3D environment with realistic physics.

### Basic World Structure

A basic Gazebo world file includes:

- Physics engine configuration
- Models with collision and visual properties
- Plugins for additional functionality
- Environment settings

### Gravity Configuration

Gravity is a fundamental aspect of physics simulation. In Gazebo, gravity is typically set to (0, 0, -9.8) m/s² to match Earth's gravitational acceleration.

## Example

Here's a basic world file with gravity configuration:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_physics">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>
    <!-- Additional world elements -->
  </world>
</sdf>
```