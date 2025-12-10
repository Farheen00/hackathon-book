# Research: Module 3 – The AI-Robot Brain (NVIDIA Isaac)

## Decision: Choice of Isaac Sim vs. other simulators
**Rationale**: NVIDIA Isaac Sim provides photorealistic rendering capabilities and synthetic data generation tools specifically optimized for AI training. It offers tight integration with Isaac ROS for perception pipelines and provides the domain randomization features necessary for effective domain transfer from simulation to reality.

**Alternatives considered**:
- Gazebo with NVIDIA rendering: Limited photorealistic capabilities compared to Isaac Sim
- Unity with NVIDIA Omniverse: More complex setup and licensing considerations for educational use
- Custom simulation environments: Would require significant development time and lack hardware acceleration

## Decision: Level of abstraction for VSLAM explanations
**Rationale**: Using Isaac ROS's pre-built perception nodes provides the right level of abstraction for educational purposes. Students can understand the concepts and configure parameters without getting bogged down in low-level implementation details, while still seeing how hardware acceleration improves performance.

**Alternatives considered**:
- Low-level implementation from scratch: Too complex for educational module
- High-level black-box usage: Would not teach important concepts about perception
- Custom wrapper libraries: Would break compatibility with official Isaac ROS documentation

## Decision: Nav2 path planning example complexity
**Rationale**: Using simplified bipedal models with clear balance constraints provides the right complexity level for educational purposes. Examples will demonstrate the key differences from wheeled navigation while remaining accessible to students.

**Alternatives considered**:
- Full humanoid robot models: Too complex for initial learning
- Simplified wheeled robot examples: Would not demonstrate bipedal-specific challenges
- Custom navigation algorithms: Would not align with standard ROS tools

## Decision: Diagram style (text-based vs. optional graphics)
**Rationale**: Primary diagrams will be text-described to meet accessibility requirements from constitution. Optional mermaid diagrams can be included for system architecture and workflow visualization, with text descriptions as required.

**Alternatives considered**:
- Static images only: Less accessible for screen readers
- Complex SVG diagrams: May not render properly in all contexts
- No diagrams: Would reduce educational value

## Decision: Section ID strategy for RAG chatbot retrieval
**Rationale**: Following consistent section ID format "ch{number}-s{number}-{topic}" ensures reliable chatbot retrieval while remaining human-readable. This format provides clear hierarchy and maintains consistency with other modules.

**Alternatives considered**:
- Random UUIDs: Not human-readable or memorable
- Complex nested IDs: Would be difficult to maintain
- No section IDs: Would break RAG functionality

## Decision: Isaac Sim Version Selection
**Rationale**: Using Isaac Sim 2023.1+ ensures compatibility with current Isaac ROS packages and provides access to latest photorealistic rendering features while maintaining stability for educational use.

**Alternatives considered**:
- Older versions: Missing important rendering and domain randomization features
- Development versions: Less stable for educational content
- Isaac Sim 2022.x: Would lack newer hardware acceleration capabilities

## Decision: Isaac ROS Integration Approach
**Rationale**: Using Isaac ROS's native perception nodes with ROS 2 Humble provides the most stable and well-documented integration approach. This ensures compatibility with official NVIDIA documentation and community support.

**Alternatives considered**:
- Custom ROS nodes wrapping Isaac libraries: More complex and less maintainable
- Direct Isaac API integration: Would not leverage ROS 2 ecosystem
- Third-party perception packages: Would not demonstrate Isaac-specific capabilities

## Decision: Testing Strategy
**Rationale**: Implementing verification of Isaac examples through both automated checks (validating Isaac Sim configurations, checking perception pipeline outputs) and manual validation (domain transfer effectiveness, navigation performance metrics).

**Alternatives considered**:
- Only automated testing: Might miss performance and usability issues
- Only manual testing: Time-consuming and inconsistent
- No testing: Would compromise quality and reliability