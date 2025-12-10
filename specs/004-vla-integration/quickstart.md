# Quickstart Guide: Module 4 – Vision-Language-Action (VLA)

## Prerequisites

Before starting this module, ensure you have:

1. **OpenAI API Access**:
   - OpenAI API key for Whisper service
   - Python 3.10+ with pip
   - Basic understanding of speech recognition concepts
   - Microphone access for voice input (for testing)

2. **ROS 2 Environment**:
   - ROS 2 Humble Hawksbill installed
   - Python 3.10+ with rclpy
   - Basic understanding of ROS 2 concepts (topics, services, actions)
   - Terminal/shell access

3. **Development Environment**:
   - Text editor or IDE with Python support
   - OpenAI Python SDK installed
   - Speech recognition libraries (speech_recognition, pyaudio)
   - Git for version control (optional but recommended)

4. **Basic Knowledge**:
   - Completion of Modules 1-3 (ROS 2, simulation, Isaac basics)
   - Understanding of natural language processing concepts
   - Familiarity with Python programming
   - Basic understanding of humanoid robot control concepts

## Setting Up Your Environment

### 1. OpenAI and Whisper Setup

```bash
# Install OpenAI Python SDK
pip install openai

# Install speech recognition libraries
pip install SpeechRecognition pyaudio

# Set your OpenAI API key as environment variable
export OPENAI_API_KEY="your-api-key-here"
```

### 2. Verify ROS 2 Installation

```bash
# Check ROS 2 installation
ros2 --version

# Verify Python packages
python3 -c "import rclpy; print('rclpy imported successfully')"
```

### 3. Voice Recognition Setup

```bash
# Test microphone access
python3 -c "import speech_recognition as sr; r = sr.Recognizer(); print('Speech recognition library loaded')"
```

## Running Your First Voice-to-Action Example

### 1. Basic Voice Command Recognition

Create a basic voice recognition script:

```python
# Example voice command recognition script
import openai
import speech_recognition as sr
from typing import Dict, Any

class VoiceToAction:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

    def listen_for_command(self) -> str:
        """Listen for and transcribe a voice command using Whisper."""
        with self.microphone as source:
            print("Listening for command...")
            audio = self.recognizer.listen(source)

        try:
            # Use OpenAI Whisper API for transcription
            transcription = openai.Audio.transcribe(
                model="whisper-1",
                file=audio
            )
            return transcription.text
        except Exception as e:
            print(f"Error transcribing audio: {e}")
            return ""

    def process_command(self, command: str) -> Dict[str, Any]:
        """Process the transcribed command and determine intent."""
        # Example: Simple command parsing
        if "move" in command.lower():
            return {"action": "move", "direction": "forward", "distance": 1.0}
        elif "stop" in command.lower():
            return {"action": "stop"}
        elif "turn" in command.lower():
            return {"action": "rotate", "angle": 90}
        else:
            return {"action": "unknown", "raw_command": command}

# Example usage
if __name__ == "__main__":
    vta = VoiceToAction(api_key="your-openai-api-key")

    # Listen for a command
    command = vta.listen_for_command()
    print(f"Recognized command: {command}")

    # Process the command
    action_plan = vta.process_command(command)
    print(f"Action plan: {action_plan}")
```

## Running Your First Cognitive Planning Example

### 1. Natural Language to ROS 2 Action Translation

Create a cognitive planning example:

```python
# cognitive_planner.py
from typing import Dict, Any, List
from dataclasses import dataclass

@dataclass
class ROSAction:
    """Represents a ROS 2 action to be executed."""
    action_type: str  # "service", "topic", "action"
    target: str       # Service/topic/action name
    parameters: Dict[str, Any]

class CognitivePlanner:
    """Translates natural language commands into ROS 2 action sequences."""

    def __init__(self):
        self.action_mappings = {
            "move_forward": self._create_move_forward_action,
            "turn_left": self._create_turn_left_action,
            "stop": self._create_stop_action,
        }

    def plan_actions(self, command: str) -> List[ROSAction]:
        """Generate a sequence of ROS actions from a natural language command."""
        # Parse the command and identify intent
        parsed_intent = self._parse_command(command)

        # Generate action sequence based on intent
        actions = []
        if parsed_intent in self.action_mappings:
            action_func = self.action_mappings[parsed_intent]
            actions.append(action_func())

        return actions

    def _parse_command(self, command: str) -> str:
        """Parse natural language command to identify intent."""
        command_lower = command.lower()

        if "move forward" in command_lower or "go forward" in command_lower:
            return "move_forward"
        elif "turn left" in command_lower or "rotate left" in command_lower:
            return "turn_left"
        elif "stop" in command_lower:
            return "stop"
        else:
            return "unknown"

    def _create_move_forward_action(self) -> ROSAction:
        """Create a ROS action for moving forward."""
        return ROSAction(
            action_type="action",
            target="/move_base",
            parameters={"distance": 1.0, "speed": 0.5}
        )

    def _create_turn_left_action(self) -> ROSAction:
        """Create a ROS action for turning left."""
        return ROSAction(
            action_type="action",
            target="/rotate",
            parameters={"angle": 90.0, "speed": 0.3}
        )

    def _create_stop_action(self) -> ROSAction:
        """Create a ROS action for stopping."""
        return ROSAction(
            action_type="service",
            target="/stop_robot",
            parameters={}
        )

# Example usage
if __name__ == "__main__":
    planner = CognitivePlanner()

    # Example commands
    commands = [
        "Please move forward slowly",
        "Turn left ninety degrees",
        "Stop the robot immediately"
    ]

    for cmd in commands:
        print(f"\nProcessing command: '{cmd}'")
        actions = planner.plan_actions(cmd)
        print(f"Generated actions: {[action.target for action in actions]}")
```

## Running Your First Capstone Integration Example

### 1. Complete VLA System Integration

Create a complete integration example:

```python
# capstone_integration.py
import asyncio
from typing import Dict, Any, List
from dataclasses import dataclass

from voice_to_action import VoiceToAction
from cognitive_planner import CognitivePlanner, ROSAction

@dataclass
class TaskExecutionResult:
    """Result of executing a task sequence."""
    success: bool
    execution_log: List[str]
    error_message: str = ""

class VLACapstoneSystem:
    """Complete Vision-Language-Action system integrating voice, planning, and execution."""

    def __init__(self, openai_api_key: str):
        self.voice_processor = VoiceToAction(openai_api_key)
        self.cognitive_planner = CognitivePlanner()
        self.execution_log = []

    async def execute_voice_command(self, command: str = None) -> TaskExecutionResult:
        """Execute a complete voice command from recognition to robot action."""

        try:
            # Step 1: Get voice command (or use provided command)
            if command is None:
                self.execution_log.append("Listening for voice command...")
                command = self.voice_processor.listen_for_command()
            else:
                self.execution_log.append(f"Using provided command: {command}")

            if not command.strip():
                return TaskExecutionResult(False, self.execution_log, "No command recognized")

            self.execution_log.append(f"Recognized command: {command}")

            # Step 2: Process command through cognitive planning
            self.execution_log.append("Planning actions for command...")
            ros_actions = self.cognitive_planner.plan_actions(command)
            self.execution_log.append(f"Planned {len(ros_actions)} actions")

            # Step 3: Execute actions (simulated in this example)
            success = await self._execute_ros_actions(ros_actions)

            if success:
                self.execution_log.append("Task completed successfully")
            else:
                self.execution_log.append("Task execution failed")

            return TaskExecutionResult(success, self.execution_log)

        except Exception as e:
            error_msg = f"Error in VLA system: {str(e)}"
            self.execution_log.append(error_msg)
            return TaskExecutionResult(False, self.execution_log, error_msg)

    async def _execute_ros_actions(self, actions: List[ROSAction]) -> bool:
        """Simulate execution of ROS actions."""
        for action in actions:
            self.execution_log.append(f"Executing {action.action_type} to {action.target}")

            # Simulate action execution
            await asyncio.sleep(0.5)  # Simulate time for action execution

            self.execution_log.append(f"Completed {action.action_type} to {action.target}")

        return True

# Example usage
async def main():
    # Initialize the complete VLA system
    vla_system = VLACapstoneSystem("your-openai-api-key")

    # Example 1: Predefined command
    print("=== Example 1: Predefined Command ===")
    result = await vla_system.execute_voice_command("Move forward slowly")
    print(f"Success: {result.success}")
    print(f"Log: {result.execution_log}")

    # Example 2: Interactive command
    print("\n=== Example 2: Interactive Command ===")
    print("Say a command when prompted...")
    result = await vla_system.execute_voice_command()
    print(f"Success: {result.success}")
    print(f"Error: {result.error_message}")

if __name__ == "__main__":
    asyncio.run(main())
```

## Troubleshooting Common Issues

### Voice Recognition Issues
- Ensure microphone permissions are granted to your application
- Check that OpenAI API key is properly set: `echo $OPENAI_API_KEY`
- Verify internet connection for Whisper API access
- Test with clear, slow speech in quiet environments

### Cognitive Planning Failures
- Verify command parsing logic matches expected vocabulary
- Check that ROS action mappings are properly defined
- Ensure action parameters match expected ROS service/message formats

### Integration Problems
- Verify all components are properly initialized before use
- Check that the complete system flow (voice → planning → execution) is correctly connected
- Monitor execution logs to identify where failures occur

## Next Steps

1. Complete Chapter 1: Voice-to-Action with OpenAI Whisper
2. Practice with the provided voice recognition examples
3. Move to Chapter 2: Cognitive Planning - Translating Natural Language to ROS 2 Actions
4. Explore Chapter 3: Capstone Integration - Autonomous Humanoid Task Execution