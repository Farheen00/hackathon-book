# Data Model: Module 3 – The AI-Robot Brain (NVIDIA Isaac)

## Content Entities

### Module
- **name**: string - The module title (e.g., "Module 3 – The AI-Robot Brain")
- **description**: string - Brief overview of the module's learning objectives
- **chapters**: array<Chapter> - List of chapters in the module
- **learning_objectives**: array<string> - What students should learn from this module
- **prerequisites**: array<string> - Knowledge required before starting this module

### Chapter
- **id**: string - Unique identifier for chatbot retrieval (e.g., "ch3-isaac-sim")
- **title**: string - Chapter title
- **description**: string - Brief overview of the chapter content
- **sections**: array<Section> - List of sections in the chapter
- **learning_outcomes**: array<string> - Specific outcomes for this chapter
- **simulation_examples**: array<SimulationExample> - Associated runnable simulation examples

### Section
- **id**: string - Unique section identifier for chatbot retrieval (e.g., "ch3-s1-isaac-envs")
- **title**: string - Section title
- **content**: string - Main content in Markdown format
- **diagram_descriptions**: array<string> - Text descriptions of diagrams
- **key_concepts**: array<string> - Important concepts covered in the section

### SimulationExample
- **id**: string - Unique identifier for the example
- **title**: string - Brief description of the example
- **type**: string - Type of simulation (e.g., "isaac-sim", "isaac-ros", "nav2", "combined")
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

## Isaac-Specific Entities

### IsaacSimEnvironment
- **name**: string - Name of the Isaac Sim environment
- **description**: string - Purpose and characteristics of the environment
- **assets**: array<string> - List of 3D assets included in the environment
- **lighting_settings**: object - Configuration for lighting and materials
- **physics_properties**: object - Physics simulation parameters
- **domain_randomization**: object - Parameters for domain randomization

### SyntheticDataset
- **name**: string - Name of the dataset
- **sensor_types**: array<string> - Types of sensors used (LiDAR, RGB, depth, etc.)
- **data_format**: string - Format of the generated data
- **labeling_scheme**: string - How data is labeled for training
- **size**: number - Number of samples in the dataset
- **quality_metrics**: object - Metrics for dataset quality

### IsaacROSNode
- **name**: string - Name of the Isaac ROS node
- **type**: string - Type of perception node (VSLAM, object detection, etc.)
- **input_topics**: array<string> - ROS topics the node subscribes to
- **output_topics**: array<string> - ROS topics the node publishes to
- **parameters**: object - Configuration parameters for the node
- **hardware_requirements**: object - GPU and compute requirements

### BipedalNavigationPlan
- **name**: string - Name of the navigation plan
- **robot_type**: string - Type of bipedal robot
- **balance_constraints**: object - Balance and stability constraints
- **step_constraints**: object - Step planning limitations
- **trajectory_type**: string - Type of trajectory (stable, dynamic, etc.)
- **safety_margin**: number - Safety margin for navigation

### PerceptionPipeline
- **name**: string - Name of the perception pipeline
- **components**: array<string> - List of perception components in the pipeline
- **input_sensors**: array<string> - Sensors feeding into the pipeline
- **processing_steps**: array<object> - Steps in the processing pipeline
- **output_types**: array<string> - Types of outputs produced
- **performance_metrics**: object - Performance metrics for the pipeline

### DomainTransferModel
- **name**: string - Name of the model
- **training_data_source**: string - Source of training data (simulated, real, mixed)
- **architecture**: string - Model architecture type
- **transfer_metrics**: object - Metrics for domain transfer effectiveness
- **validation_results**: object - Results from validation testing

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
- Content must align with official Isaac Sim/ROS documentation

### Simulation Example Validation
- Must run without modification in specified Isaac Sim/ROS environment
- Must include proper error handling and documentation
- Must be accompanied by expected output description
- Must align with runnable code standards from constitution

### Isaac Sim Environment Validation
- Must generate realistic sensor data matching real-world patterns
- Must support domain randomization for effective transfer
- Must be configurable for different learning scenarios
- Must include proper lighting and physics properties

## Relationships

- Module **contains** 1..n Chapters
- Chapter **contains** 1..n Sections
- Chapter **contains** 0..n SimulationExamples
- Section **may reference** 0..n Diagrams
- SimulationExample **belongs to** exactly 1 Chapter
- Diagram **may be referenced by** 0..n Sections
- IsaacSimEnvironment **contains** 1..n SyntheticDatasets
- IsaacROSNode **is part of** 1..n PerceptionPipelines
- BipedalNavigationPlan **is used by** 1..n SimulationExamples
- PerceptionPipeline **is part of** 1..n SimulationExamples
- DomainTransferModel **is trained with** 1..n SyntheticDatasets

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