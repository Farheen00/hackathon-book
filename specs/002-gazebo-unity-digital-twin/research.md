# Research: Module 2 – The Digital Twin (Gazebo & Unity)

## Decision: Gazebo vs. Unity Example Selection
**Rationale**: Using Gazebo for physics simulation and Unity for high-fidelity visualization represents the industry standard approach for digital twin development. Gazebo provides accurate physics simulation based on ODE, Bullet, or DART physics engines, while Unity offers superior rendering capabilities and user experience.

**Alternatives considered**:
- Using only Gazebo for both physics and visualization: Limited visual fidelity
- Using only Unity for both: Less accurate physics simulation without proper physics engine integration
- Other simulation platforms (Isaac Sim): More complex setup and licensing considerations

## Decision: Sensor Simulation Formats (LiDAR, Depth, IMU)
**Rationale**: Following ROS 2 standard message types for sensor simulation to ensure compatibility with existing robotics frameworks and tools:
- LiDAR: sensor_msgs/PointCloud2 for point cloud data
- Depth: sensor_msgs/Image with 32-bit float encoding for depth values
- IMU: sensor_msgs/Imu with orientation, angular velocity, and linear acceleration

**Alternatives considered**:
- Custom formats: Would break compatibility with existing tools
- Raw binary formats: Less accessible for educational purposes
- Different ROS message types: Would complicate integration with existing ROS tools

## Decision: Code Block Conventions and Environment Setup Notes
**Rationale**: Using standard ROS 2 workspace structure with clear environment setup instructions. Code examples will follow ROS 2 best practices with proper error handling and documentation. Environment setup will include both Gazebo and Unity installation procedures with version compatibility notes.

**Alternatives considered**:
- Containerized environments (Docker): More complex for beginners
- Cloud-based simulation: Less control and potential connectivity issues
- Different build systems: ROS 2 colcon is the standard for ROS 2 packages

## Decision: Diagram Style (text + optional mermaid)
**Rationale**: Primary diagrams will be text-described to meet accessibility requirements from constitution. Optional mermaid diagrams can be included for system architecture and workflow visualization, with text descriptions as required.

**Alternatives considered**:
- Static images only: Less accessible for screen readers
- Complex SVG diagrams: May not render properly in all contexts
- No diagrams: Would reduce educational value

## Decision: Gazebo Version Selection
**Rationale**: Using Gazebo Garden or Harmonic (latest stable versions) to ensure compatibility with ROS 2 Humble/Humble and provide access to latest features while maintaining stability for educational use.

**Alternatives considered**:
- Gazebo Classic: Being phased out in favor of new Gazebo
- Older versions: Missing important features and bug fixes
- Development versions: Less stable for educational content

## Decision: Unity Version Selection
**Rationale**: Using Unity 2022.3 LTS (Long Term Support) to ensure stability and long-term compatibility for educational content. This version provides the latest features while being supported for an extended period.

**Alternatives considered**:
- Newest Unity version: May have stability issues
- Older LTS versions: Missing important educational features
- Unity Personal vs. Pro: Personal is sufficient for educational examples

## Decision: Integration Approach
**Rationale**: Using ROS 2 as the middleware to connect Gazebo physics simulation with Unity visualization, allowing students to understand both systems while maintaining the ability to use them independently.

**Alternatives considered**:
- Direct Unity-ROS integration: More complex setup
- Standalone simulation in each platform: Less comprehensive learning
- Custom middleware: Would complicate the learning process

## Decision: Testing Strategy
**Rationale**: Implementing verification of simulation examples through both automated checks (validating URDF files, checking ROS message formats) and manual validation (visual inspection of simulation behavior, sensor output verification).

**Alternatives considered**:
- Only automated testing: Might miss visual/behavioral issues
- Only manual testing: Time-consuming and inconsistent
- No testing: Would compromise quality and reliability