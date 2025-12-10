# Data Model: Module 2 – The Digital Twin (Gazebo & Unity)

## Content Entities

### Module
- **name**: string - The module title (e.g., "Module 2 – The Digital Twin")
- **description**: string - Brief overview of the module's learning objectives
- **chapters**: array<Chapter> - List of chapters in the module
- **learning_objectives**: array<string> - What students should learn from this module
- **prerequisites**: array<string> - Knowledge required before starting this module

### Chapter
- **id**: string - Unique identifier for chatbot retrieval (e.g., "ch2-gazebo-physics")
- **title**: string - Chapter title
- **description**: string - Brief overview of the chapter content
- **sections**: array<Section> - List of sections in the chapter
- **learning_outcomes**: array<string> - Specific outcomes for this chapter
- **simulation_examples**: array<SimulationExample> - Associated runnable simulation examples

### Section
- **id**: string - Unique section identifier for chatbot retrieval (e.g., "ch2-s1-physics-worlds")
- **title**: string - Section title
- **content**: string - Main content in Markdown format
- **diagram_descriptions**: array<string> - Text descriptions of diagrams
- **key_concepts**: array<string> - Important concepts covered in the section

### SimulationExample
- **id**: string - Unique identifier for the example
- **title**: string - Brief description of the example
- **type**: string - Type of simulation (e.g., "gazebo", "unity", "sensor")
- **description**: string - Explanation of what the simulation demonstrates
- **expected_output**: string - What the simulation should produce when run
- **prerequisites**: array<string> - What needs to be set up before running
- **files**: array<string> - List of files that make up the simulation

### Diagram
- **id**: string - Unique identifier
- **title**: string - Brief description
- **text_description**: string - Detailed text description for accessibility
- **type**: string - Type of diagram (e.g., "architecture", "workflow", "simulation")
- **mermaid_code**: string - Optional mermaid syntax for visual representation

## Simulation-Specific Entities

### GazeboWorld
- **name**: string - Name of the Gazebo world
- **gravity**: object - Gravity vector (x, y, z components)
- **models**: array<GazeboModel> - Models included in the world
- **plugins**: array<GazeboPlugin> - Plugins loaded in the world
- **physics_engine**: string - Physics engine used (ODE, Bullet, DART)

### GazeboModel
- **name**: string - Model name
- **pose**: object - Position and orientation (x, y, z, roll, pitch, yaw)
- **static**: boolean - Whether the model is static
- **link**: GazeboLink - The link definition
- **joint**: array<GazeboJoint> - Joints connected to this model

### GazeboLink
- **name**: string - Link name
- **inertial**: object - Mass and inertia properties
- **visual**: GazeboVisual - Visual properties
- **collision**: GazeboCollision - Collision properties

### GazeboSensor
- **name**: string - Sensor name
- **type**: string - Sensor type (e.g., "lidar", "camera", "imu")
- **topic**: string - ROS topic for sensor data
- **update_rate**: number - Update rate in Hz
- **sensor_specific_params**: object - Type-specific parameters

### UnityScene
- **name**: string - Scene name
- **objects**: array<UnityObject> - Objects in the scene
- **lighting**: UnityLighting - Lighting configuration
- **materials**: array<UnityMaterial> - Materials used in the scene
- **rendering_settings**: object - Rendering quality settings

### SensorData
- **sensor_type**: string - Type of sensor (lidar, depth, imu)
- **timestamp**: number - Timestamp of the data
- **frame_id**: string - Coordinate frame of the data
- **data**: object - The actual sensor data in appropriate format
- **format**: string - Data format (e.g., "PointCloud2", "Image", "Imu")

## Validation Rules

### Module Validation
- Module title must be unique within the book
- Must have at least one chapter
- Learning objectives must align with constitution's educational clarity principle
- Prerequisites must be clearly defined and verifiable

### Chapter Validation
- Chapter ID must follow format: "ch{number}-{topic}" for consistent retrieval
- Must include at least one section
- Simulation examples must be runnable and tested in appropriate environments
- Must include section IDs for chatbot retrieval

### Section Validation
- Section ID must follow format: "ch{number}-s{number}-{topic}" for consistent retrieval
- Content must maintain grade 10-12 reading level
- All diagrams must have text descriptions for accessibility
- Content must align with official Gazebo/Unity documentation

### Simulation Example Validation
- Must run without modification in specified Gazebo/Unity environment
- Must include proper error handling and documentation
- Must be accompanied by expected output description
- Must align with runnable code standards from constitution

### Sensor Data Validation
- Must follow ROS 2 standard message formats
- Data must match expected real-world sensor patterns
- Timestamps must be properly synchronized
- Coordinate frames must be correctly specified

## Relationships

- Module **contains** 1..n Chapters
- Chapter **contains** 1..n Sections
- Chapter **contains** 0..n SimulationExamples
- Section **may reference** 0..n Diagrams
- SimulationExample **belongs to** exactly 1 Chapter
- Diagram **may be referenced by** 0..n Sections
- GazeboWorld **contains** 1..n GazeboModels
- GazeboModel **has** 1..n GazeboLinks and 0..n GazeboJoints
- UnityScene **contains** 1..n UnityObjects
- SensorData **is generated by** Sensor

## State Transitions (for content development)

### Content Creation Flow
1. **Draft** → Content is being written
2. **Reviewed** → Content has been reviewed for technical accuracy
3. **Validated** → Simulation examples tested and diagrams verified
4. **Published** → Content is ready for deployment

### Validation Requirements by State
- **Draft**: Basic structure and content in place
- **Reviewed**: Meets educational clarity and technical accuracy standards
- **Validated**: All simulation examples run successfully, diagrams accessible
- **Published**: Passes all Docusaurus build checks and deployment validation