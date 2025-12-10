---
sidebar_position: 2
title: Action Translation
description: Translating natural language commands to ROS 2 action sequences for robot execution
id: ch4-s5-action-translation
---

# Action Translation

This section covers translating natural language commands to ROS 2 action sequences for robot execution. You'll learn how to convert parsed natural language into executable ROS 2 actions and service calls.

## Key Concepts

- Natural language to ROS 2 action mapping
- Action sequence generation
- Parameter extraction and validation
- Service and topic mapping for robotic actions

## Diagram Descriptions

1. **Action Translation Pipeline**: A flowchart showing the transformation from natural language commands to ROS 2 action sequences.
2. **ROS 2 Action Mapping**: A diagram illustrating how natural language concepts map to specific ROS 2 actions, services, and topics.

## Content

Action translation involves mapping the structured data from natural language processing to specific ROS 2 actions that can be executed by the robot. This includes:

- Identifying appropriate ROS 2 action servers
- Extracting parameters for action goals
- Mapping spatial relationships to coordinate frames
- Converting qualitative descriptions to quantitative values

### Action Mapping Strategies

Common mapping strategies include:
- Template-based mapping (predefined command patterns)
- Semantic mapping (mapping based on meaning)
- Context-aware mapping (considering environmental context)
- Learning-based mapping (adapting from experience)

### Parameter Extraction

Parameters must be extracted from natural language and converted to appropriate ROS 2 message types:
- Coordinates (positions, orientations)
- Velocities and accelerations
- Object identifiers and properties
- Duration and timing constraints

## Example

Here's an example of action translation from natural language to ROS 2 actions:

```python
from typing import Dict, List, Any
import math

class ActionTranslator:
    def __init__(self):
        # Define action mappings
        self.action_mappings = {
            'move': '/move_base',
            'navigate': '/navigate_to_pose',
            'pick': '/pick_object',
            'place': '/place_object',
            'grip': '/gripper_command',
            'turn': '/rotate_robot',
            'stop': '/stop_robot'
        }

        # Define parameter mappings
        self.parameter_mappings = {
            'distance': ['meters', 'units', 'forward', 'backward'],
            'angle': ['degrees', 'radians', 'turn', 'rotate'],
            'velocity': ['speed', 'fast', 'slow', 'quickly'],
            'object': ['object', 'item', 'thing', 'it']
        }

    def translate_to_ros_actions(self, parsed_command: Dict) -> List[Dict[str, Any]]:
        """Translate parsed command to sequence of ROS 2 actions"""
        actions = []

        action_verb = parsed_command.get('action')
        if not action_verb:
            return actions

        # Map action to ROS 2 action
        ros_action = self.action_mappings.get(action_verb)
        if not ros_action:
            # If no direct mapping, try semantic matching
            ros_action = self._semantic_match_action(action_verb)

        if ros_action:
            # Extract parameters for the action
            params = self._extract_parameters(parsed_command)

            action_dict = {
                'action_server': ros_action,
                'parameters': params,
                'message_type': self._get_message_type(ros_action),
                'goal_id': self._generate_goal_id()
            }

            actions.append(action_dict)

        return actions

    def _semantic_match_action(self, action_verb: str) -> str:
        """Semantically match action verb to appropriate ROS 2 action"""
        # Example semantic matching
        if action_verb in ['go', 'move', 'travel']:
            return '/move_base'
        elif action_verb in ['rotate', 'turn', 'spin']:
            return '/rotate_robot'
        elif action_verb in ['stop', 'halt', 'pause']:
            return '/stop_robot'
        else:
            return None

    def _extract_parameters(self, parsed_command: Dict) -> Dict[str, Any]:
        """Extract and convert parameters from parsed command"""
        params = {}

        # Extract distance parameters
        for obj in parsed_command.get('objects', []):
            text = obj['text'].lower()
            if any(keyword in text for keyword in ['meter', 'unit']):
                try:
                    distance = float(text.replace('meters', '').strip())
                    params['distance'] = distance
                except ValueError:
                    pass

        # Extract angle parameters
        for modifier in parsed_command.get('modifiers', []):
            if 'degree' in modifier.lower():
                try:
                    angle = float(modifier.replace('degrees', '').strip())
                    params['angle'] = math.radians(angle)
                except ValueError:
                    pass

        # Extract spatial relations
        for relation in parsed_command.get('spatial_relations', []):
            if relation == 'near':
                params['approach'] = 'close'
            elif relation == 'far':
                params['approach'] = 'distant'

        return params

    def _get_message_type(self, action_server: str) -> str:
        """Get appropriate message type for action server"""
        message_types = {
            '/move_base': 'geometry_msgs/PoseStamped',
            '/navigate_to_pose': 'nav_msgs/MoveBaseGoal',
            '/pick_object': 'object_manipulation_msgs/PickUpObject',
            '/place_object': 'object_manipulation_msgs/PlaceObject',
            '/gripper_command': 'control_msgs/GripperCommand',
            '/rotate_robot': 'geometry_msgs/Twist',
            '/stop_robot': 'std_msgs/Empty'
        }
        return message_types.get(action_server, 'unknown')

    def _generate_goal_id(self) -> str:
        """Generate unique goal ID"""
        import uuid
        return str(uuid.uuid4())
```