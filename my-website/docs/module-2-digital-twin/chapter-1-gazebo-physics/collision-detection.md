---
sidebar_position: 2
title: Collision Detection
description: Setting up collision models and detection in Gazebo for humanoid robots
id: ch2-s2-collision-detection
---

# Collision Detection

This section covers setting up collision models and detection in Gazebo for humanoid robots. You'll learn how to configure collision properties that enable realistic robot-environment interactions.

## Key Concepts

- Collision geometry types (box, sphere, cylinder, mesh)
- Collision properties and materials
- Contact detection
- Collision response parameters

## Diagram Descriptions

1. **Collision Geometry Types**: A visual comparison showing different collision geometry types (box, sphere, cylinder, mesh) and their applications.
2. **Collision Detection Pipeline**: A flow diagram showing how collision detection works from geometry definition to contact response.

## Content

Collision detection is crucial for realistic physics simulation. In Gazebo, collision detection is handled through collision elements in model definitions.

### Collision Geometry

Gazebo supports several collision geometry types:

- Box: Rectangular solid shapes
- Sphere: Perfect spheres
- Cylinder: Cylindrical shapes
- Mesh: Complex shapes defined by triangular meshes
- Plane: Infinite flat surfaces

### Collision Properties

Collision properties define how objects interact when they come into contact:

- Friction coefficients
- Bounce parameters
- Surface contact properties

## Example

Here's how to define collision properties for a robot link:

```xml
<link name="link">
  <collision name="collision">
    <geometry>
      <box>
        <size>0.2 0.2 0.2</size>
      </box>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
        </ode>
      </friction>
    </surface>
  </collision>
</link>
```