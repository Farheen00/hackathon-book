# Data Model: Module 4 – Vision-Language-Action (VLA)

## Content Entities

### Module
- **name**: string - The module title (e.g., "Module 4 – Vision-Language-Action (VLA)")
- **description**: string - Brief overview of the module's learning objectives
- **chapters**: array<Chapter> - List of chapters in the module
- **learning_objectives**: array<string> - What students should learn from this module
- **prerequisites**: array<string> - Knowledge required before starting this module

### Chapter
- **id**: string - Unique identifier for chatbot retrieval (e.g., "ch4-whisper-v2a")
- **title**: string - Chapter title
- **description**: string - Brief overview of the chapter content
- **sections**: array<Section> - List of sections in the chapter
- **learning_outcomes**: array<string> - Specific outcomes for this chapter
- **vla_examples**: array<VLAExample> - Associated runnable VLA examples

### Section
- **id**: string - Unique section identifier for chatbot retrieval (e.g., "ch4-s1-whisper-integration")
- **title**: string - Section title
- **content**: string - Main content in Markdown format
- **diagram_descriptions**: array<string> - Text descriptions of diagrams
- **key_concepts**: array<string> - Important concepts covered in the section

### VLAExample
- **id**: string - Unique identifier for the example
- **title**: string - Brief description of the example
- **type**: string - Type of VLA example (e.g., "whisper", "cognitive-planning", "integration")
- **description**: string - Explanation of what the VLA example demonstrates
- **expected_output**: string - What the VLA example should produce when run
- **prerequisites**: array<string> - What needs to be set up before running
- **files**: array<string> - List of files that make up the VLA example

### Diagram
- **id**: string - Unique identifier
- **title**: string - Brief description
- **text_description**: string - Detailed text description for accessibility
- **type**: string - Type of diagram (e.g., "architecture", "workflow", "v2a")
- **mermaid_code**: string - Optional mermaid syntax for visual representation

## VLA-Specific Entities

### VoiceCommand
- **transcription**: string - The recognized text from Whisper
- **intent**: string - The interpreted purpose of the command
- **confidence**: number - Confidence score of the recognition (0-1)
- **timestamp**: number - When the command was captured
- **processing_result**: object - Result of natural language processing

### CognitivePlan
- **sequence_id**: string - Identifier for the action sequence
- **steps**: array<CognitiveStep> - Ordered list of actions to execute
- **input_command**: string - Original natural language command
- **parsed_structure**: object - Structured representation of the command
- **execution_context**: object - Environment and robot state context

### CognitiveStep
- **step_id**: string - Unique identifier for the step
- **action_type**: string - Type of ROS 2 action (service call, topic publish, action goal)
- **parameters**: object - Parameters for the action
- **dependencies**: array<string> - Other steps this step depends on
- **estimated_duration**: number - Expected time to execute (seconds)

### ActionSequence
- **sequence_id**: string - Identifier for the sequence
- **name**: string - Human-readable name
- **ros_actions**: array<ROSAction> - List of ROS 2 actions to execute
- **validation_criteria**: object - Conditions for successful execution
- **error_handling**: object - Procedures for handling failures

### ROSAction
- **action_id**: string - Unique identifier for the ROS action
- **type**: string - Type: "service", "topic", or "action"
- **target**: string - Service/topic/action name
- **request_data**: object - Data to send (for services/actions)
- **message_data**: object - Message content (for topics)
- **timeout**: number - Maximum time to wait for response

### VLAIntegration
- **name**: string - Name of the integration example
- **components**: array<string> - List of VLA components involved
- **workflow**: object - Complete workflow from voice input to robot action
- **validation_metrics**: object - Metrics for measuring success
- **failure_modes**: array<object> - Potential failure scenarios and recovery

## Validation Rules

### Module Validation
- Module title must be unique within the book
- Must have at least one chapter
- Learning objectives must align with constitution's educational clarity principle
- Prerequisites must be clearly defined and verifiable

### Chapter Validation
- Chapter ID must follow format: "ch{number}-{topic}" for consistent retrieval
- Must include at least one section
- VLA examples must be runnable and tested in appropriate environments
- Must include section IDs for chatbot retrieval

### Section Validation
- Section ID must follow format: "ch{number}-s{number}-{topic}" for consistent retrieval
- Content must maintain grade 10-12 reading level
- All diagrams must have text descriptions for accessibility
- Content must align with official OpenAI/ROS documentation

### VLA Example Validation
- Must run without modification in specified VLA environment
- Must include proper error handling and documentation
- Must be accompanied by expected output description
- Must align with runnable code standards from constitution

### Voice Command Validation
- Must include proper confidence scoring
- Must handle ambiguous or unrecognized commands gracefully
- Must provide feedback to user about recognition status
- Must follow privacy and security best practices

## Relationships

- Module **contains** 1..n Chapters
- Chapter **contains** 1..n Sections
- Chapter **contains** 0..n VLAExamples
- Section **may reference** 0..n Diagrams
- VLAExample **belongs to** exactly 1 Chapter
- Diagram **may be referenced by** 0..n Sections
- VoiceCommand **generates** 1 CognitivePlan
- CognitivePlan **contains** 1..n CognitiveSteps
- CognitiveStep **executes** 1 ROSAction
- ActionSequence **comprises** 1..n ROSActions
- VLAIntegration **includes** 1..n ActionSequences

## State Transitions (for content development)

### Content Creation Flow
1. **Draft** → Content is being written
2. **Reviewed** → Content has been reviewed for technical accuracy
3. **Validated** → VLA examples tested and diagrams verified
4. **Published** → Content is ready for deployment

### Validation Requirements by State
- **Draft**: Basic structure and content in place
- **Reviewed**: Meets educational clarity and technical accuracy standards
- **Validated**: All VLA examples run successfully, diagrams accessible
- **Published**: Passes all Docusaurus build checks and deployment validation