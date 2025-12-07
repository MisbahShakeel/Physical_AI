#!/usr/bin/env python3
"""
Voice Command Parser for Robot Control

This script provides natural language processing for voice command interpretation,
mapping spoken commands to robot actions with context awareness and validation.
"""

import re
from typing import Dict, List, Tuple, Optional, Any
import logging


class VoiceCommandParser:
    """
    Natural language processor for voice command interpretation
    """

    def __init__(self):
        # Define command patterns with associated actions
        self.command_patterns = {
            # Movement commands
            'move_forward': {
                'patterns': [
                    r'move forward',
                    r'go forward',
                    r'forward',
                    r'go ahead',
                    r'move straight',
                    r'go straight'
                ],
                'action': 'move_forward',
                'params': {'direction': 'forward', 'distance': 1.0}
            },
            'move_backward': {
                'patterns': [
                    r'move backward',
                    r'go backward',
                    r'backward',
                    r'back',
                    r'go back',
                    r'move back'
                ],
                'action': 'move_backward',
                'params': {'direction': 'backward', 'distance': 1.0}
            },
            'turn_left': {
                'patterns': [
                    r'turn left',
                    r'left turn',
                    r'turn to the left',
                    r'pivot left',
                    r'rotate left'
                ],
                'action': 'turn_left',
                'params': {'direction': 'left', 'angle': 90.0}
            },
            'turn_right': {
                'patterns': [
                    r'turn right',
                    r'right turn',
                    r'turn to the right',
                    r'pivot right',
                    r'rotate right'
                ],
                'action': 'turn_right',
                'params': {'direction': 'right', 'angle': 90.0}
            },
            'move_left': {
                'patterns': [
                    r'move left',
                    r'strafe left',
                    r'go left',
                    r'move to the left'
                ],
                'action': 'move_left',
                'params': {'direction': 'left', 'distance': 1.0}
            },
            'move_right': {
                'patterns': [
                    r'move right',
                    r'strafe right',
                    r'go right',
                    r'move to the right'
                ],
                'action': 'move_right',
                'params': {'direction': 'right', 'distance': 1.0}
            },
            'stop': {
                'patterns': [
                    r'stop',
                    r'halt',
                    r'freeze',
                    r'pause',
                    r'cease',
                    r'quit'
                ],
                'action': 'stop',
                'params': {}
            },
            'dance': {
                'patterns': [
                    r'dance',
                    r'dance for me',
                    r'show me a dance',
                    r'perform a dance'
                ],
                'action': 'dance',
                'params': {}
            },
            'wave': {
                'patterns': [
                    r'wave',
                    r'wave hello',
                    r'wave goodbye',
                    r'wave your hand',
                    r'waggle your hand'
                ],
                'action': 'wave',
                'params': {}
            },
            'greet': {
                'patterns': [
                    r'hello',
                    r'hi',
                    r'greetings',
                    r'good morning',
                    r'good afternoon',
                    r'good evening'
                ],
                'action': 'greet',
                'params': {}
            },
            'goodbye': {
                'patterns': [
                    r'goodbye',
                    r'bye',
                    r'see you',
                    r'farewell',
                    r'thank you'
                ],
                'action': 'goodbye',
                'params': {}
            },
            'help': {
                'patterns': [
                    r'help',
                    r'what can you do',
                    r'what are your capabilities',
                    r'how do i use you',
                    r'assist me'
                ],
                'action': 'help',
                'params': {}
            }
        }

        # Distance/angle modifiers
        self.distance_patterns = [
            (r'(\d+(?:\.\d+)?)\s*steps?', lambda m: float(m.group(1))),
            (r'(\d+(?:\.\d+)?)\s*meters?', lambda m: float(m.group(1))),
            (r'(\d+(?:\.\d+)?)\s*feet', lambda m: float(m.group(1)) * 0.3048),
        ]

        self.angle_patterns = [
            (r'(\d+(?:\.\d+)?)\s*degrees?', lambda m: float(m.group(1))),
            (r'(\d+(?:\.\d+)?)\s*degrees?', lambda m: float(m.group(1))),
        ]

        # Context variables
        self.context = {
            'last_action': None,
            'robot_position': (0, 0, 0),
            'robot_orientation': 0,
            'user_preferences': {}
        }

        # Set up logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

    def parse_command(self, command_text: str) -> Optional[Dict[str, Any]]:
        """
        Parse a voice command and return the corresponding action

        Args:
            command_text: The raw voice command text

        Returns:
            Dictionary with action information, or None if command not recognized
        """
        original_command = command_text
        command_text = command_text.lower().strip()

        # Extract distance and angle information before matching patterns
        extracted_params = self._extract_numerical_params(command_text)
        command_text = self._remove_numerical_expressions(command_text)

        # Check each command pattern
        for action_name, action_info in self.command_patterns.items():
            for pattern in action_info['patterns']:
                if re.search(pattern, command_text):
                    # Create result with action info and extracted parameters
                    result = {
                        'action': action_info['action'],
                        'original_command': original_command,
                        'confidence': 0.9,  # High confidence for exact matches
                        'params': action_info['params'].copy()
                    }

                    # Update parameters with extracted values
                    result['params'].update(extracted_params)

                    # Log the parsed command
                    self.logger.info(f"Parsed command: {action_name} with params {result['params']}")
                    return result

        # If no pattern matches, try fuzzy matching
        fuzzy_result = self._fuzzy_match(command_text)
        if fuzzy_result:
            fuzzy_result['params'].update(extracted_params)
            return fuzzy_result

        # Command not recognized
        self.logger.warning(f"Command not recognized: {command_text}")
        return None

    def _extract_numerical_params(self, command_text: str) -> Dict[str, float]:
        """
        Extract distance and angle parameters from command text

        Args:
            command_text: The command text to analyze

        Returns:
            Dictionary with extracted parameters
        """
        params = {}

        # Extract distances
        for pattern, converter in self.distance_patterns:
            matches = re.findall(pattern, command_text)
            if matches:
                try:
                    params['distance'] = converter(matches[0])
                except:
                    pass  # Ignore conversion errors

        # Extract angles
        for pattern, converter in self.angle_patterns:
            matches = re.findall(pattern, command_text)
            if matches:
                try:
                    params['angle'] = converter(matches[0])
                except:
                    pass  # Ignore conversion errors

        return params

    def _remove_numerical_expressions(self, command_text: str) -> str:
        """
        Remove numerical expressions from command text for pattern matching

        Args:
            command_text: The command text to clean

        Returns:
            Cleaned command text
        """
        # Remove distances
        for pattern, _ in self.distance_patterns:
            command_text = re.sub(pattern, '', command_text)

        # Remove angles
        for pattern, _ in self.angle_patterns:
            command_text = re.sub(pattern, '', command_text)

        # Clean up extra spaces
        command_text = re.sub(r'\s+', ' ', command_text).strip()

        return command_text

    def _fuzzy_match(self, command_text: str) -> Optional[Dict[str, Any]]:
        """
        Perform fuzzy matching for commands that don't match exactly

        Args:
            command_text: The command text to fuzzy match

        Returns:
            Dictionary with action information if fuzzy match found, None otherwise
        """
        # This is a simplified fuzzy matching implementation
        # In a real system, you might use more sophisticated NLP techniques

        # Look for keywords that might indicate intent
        if 'forward' in command_text or 'ahead' in command_text:
            return {
                'action': 'move_forward',
                'original_command': command_text,
                'confidence': 0.7,
                'params': {'direction': 'forward', 'distance': 1.0}
            }

        if 'back' in command_text or 'backward' in command_text:
            return {
                'action': 'move_backward',
                'original_command': command_text,
                'confidence': 0.7,
                'params': {'direction': 'backward', 'distance': 1.0}
            }

        if 'left' in command_text:
            return {
                'action': 'turn_left',
                'original_command': command_text,
                'confidence': 0.7,
                'params': {'direction': 'left', 'angle': 90.0}
            }

        if 'right' in command_text:
            return {
                'action': 'turn_right',
                'original_command': command_text,
                'confidence': 0.7,
                'params': {'direction': 'right', 'angle': 90.0}
            }

        if 'stop' in command_text or 'halt' in command_text:
            return {
                'action': 'stop',
                'original_command': command_text,
                'confidence': 0.7,
                'params': {}
            }

        # No fuzzy match found
        return None

    def validate_command(self, command_result: Dict[str, Any]) -> bool:
        """
        Validate a parsed command for safety and feasibility

        Args:
            command_result: The parsed command result

        Returns:
            True if command is valid, False otherwise
        """
        action = command_result['action']
        params = command_result['params']

        # Validate distance parameters
        if 'distance' in params:
            distance = params['distance']
            if distance <= 0 or distance > 10.0:  # Max 10 meters for safety
                self.logger.warning(f"Invalid distance: {distance}")
                return False

        # Validate angle parameters
        if 'angle' in params:
            angle = params['angle']
            if abs(angle) > 360:  # Max 360 degrees
                self.logger.warning(f"Invalid angle: {angle}")
                return False

        # Additional validations can be added here

        return True

    def update_context(self, command_result: Dict[str, Any]):
        """
        Update the context based on the executed command

        Args:
            command_result: The command result that was executed
        """
        self.context['last_action'] = command_result['action']

        # Update position and orientation based on action
        action = command_result['action']
        params = command_result['params']

        if action == 'move_forward':
            # Update position based on current orientation
            pass
        elif action == 'turn_left':
            self.context['robot_orientation'] += params.get('angle', 90.0)
        elif action == 'turn_right':
            self.context['robot_orientation'] -= params.get('angle', 90.0)

        # Normalize orientation to 0-360 range
        self.context['robot_orientation'] %= 360


def main():
    """
    Main function to demonstrate the voice command parser
    """
    parser = VoiceCommandParser()

    # Test commands
    test_commands = [
        "move forward 2 meters",
        "turn left 45 degrees",
        "stop immediately",
        "dance for me",
        "wave hello",
        "go back 3 steps",
        "turn right"
    ]

    print("Voice Command Parser Demo")
    print("="*40)

    for command in test_commands:
        print(f"\nProcessing: '{command}'")
        result = parser.parse_command(command)

        if result:
            print(f"  Action: {result['action']}")
            print(f"  Params: {result['params']}")
            print(f"  Confidence: {result['confidence']}")

            # Validate command
            is_valid = parser.validate_command(result)
            print(f"  Valid: {is_valid}")

            # Update context
            if is_valid:
                parser.update_context(result)
        else:
            print("  Command not recognized")


if __name__ == "__main__":
    main()