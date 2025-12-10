---
sidebar_position: 2
title: Command Recognition
description: Processing voice commands and recognizing intents using Whisper and NLP
id: ch4-s2-command-recognition
---

# Command Recognition

This section covers processing voice commands and recognizing intents using Whisper and Natural Language Processing techniques. You'll learn how to take transcribed text from Whisper and extract actionable commands for robotic systems.

## Key Concepts

- Intent recognition from transcribed text
- Command categorization and validation
- Confidence scoring for recognition accuracy
- Error handling for unrecognized commands

## Diagram Descriptions

1. **Command Recognition Pipeline**: A flow diagram showing the process from Whisper transcription to intent recognition and command categorization.
2. **Intent Classification Tree**: A hierarchical representation of different command categories and subcategories for robotic control.

## Content

Command recognition involves taking the transcribed text from Whisper and determining the user's intent. This requires natural language processing techniques to identify key phrases and map them to specific robot actions.

### Intent Recognition

Common robotic command intents include:
- Movement commands (move forward, turn left, etc.)
- Action commands (pick up object, open door, etc.)
- Query commands (where am I, what time is it, etc.)
- Control commands (stop, pause, resume, etc.)

### Confidence Scoring

Each recognized command should include a confidence score to indicate how certain the system is about the interpretation. This helps in error handling and allows for appropriate user feedback.

## Example

Here's an example of command recognition implementation:

```python
class CommandRecognizer:
    def __init__(self):
        self.command_patterns = {
            'move_forward': [r'move forward', r'go forward', r'forward'],
            'turn_left': [r'turn left', r'rotate left', r'left'],
            'turn_right': [r'turn right', r'rotate right', r'right'],
            'stop': [r'stop', r'halt', r'freeze'],
            'greet': [r'hello', r'hi', r'hey']
        }

    def recognize_intent(self, transcribed_text):
        """Recognize intent from transcribed text"""
        text_lower = transcribed_text.lower()

        for intent, patterns in self.command_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text_lower):
                    # Calculate confidence based on match strength
                    confidence = 0.9 if re.fullmatch(pattern, text_lower) else 0.7
                    return {
                        'intent': intent,
                        'confidence': confidence,
                        'original_text': transcribed_text
                    }

        # If no pattern matched, return unknown
        return {
            'intent': 'unknown',
            'confidence': 0.0,
            'original_text': transcribed_text
        }
```