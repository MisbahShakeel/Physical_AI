#!/usr/bin/env python3
"""
LLM Task Planner for ROS 2 Action Sequencing

This script implements a system that uses Large Language Models (LLMs)
to generate task plans for ROS 2 robots based on natural language commands.
"""

import openai
import os
import json
import re
import time
import logging
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass
import yaml


@dataclass
class RobotCapability:
    """Represents a robot's capabilities"""
    name: str
    parameters: List[str]
    description: str


@dataclass
class TaskPlan:
    """Represents a planned task sequence"""
    command: str
    actions: List[Dict[str, Any]]
    confidence: float
    execution_time: float


class LLMAgent:
    """
    A class to interact with Large Language Models for task planning
    """

    def __init__(self, api_key: Optional[str] = None, model: str = "gpt-3.5-turbo"):
        """
        Initialize the LLM agent

        Args:
            api_key: OpenAI API key (if not set, looks for OPENAI_API_KEY environment variable)
            model: LLM model to use (default: gpt-3.5-turbo)
        """
        # Set API key
        if api_key:
            openai.api_key = api_key
        elif os.getenv("OPENAI_API_KEY"):
            openai.api_key = os.getenv("OPENAI_API_KEY")
        else:
            # For demonstration purposes, we'll use a mock implementation
            logging.warning("No OpenAI API key found. Using mock implementation.")
            self.use_mock = True
        self.use_mock = True  # Set to True for demo without API key

        self.model = model
        self.robot_capabilities = self._define_robot_capabilities()

        # Set up logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

    def _define_robot_capabilities(self) -> List[RobotCapability]:
        """
        Define the robot's capabilities for planning

        Returns:
            List of robot capabilities
        """
        capabilities = [
            RobotCapability(
                name="move_to",
                parameters=["x", "y", "z"],
                description="Move the robot to a specific location (x, y, z coordinates)"
            ),
            RobotCapability(
                name="pick_object",
                parameters=["object_name", "location"],
                description="Pick up an object by name from a specific location"
            ),
            RobotCapability(
                name="place_object",
                parameters=["object_name", "location"],
                description="Place an object by name at a specific location"
            ),
            RobotCapability(
                name="grasp",
                parameters=["object_name"],
                description="Grasp an object by name"
            ),
            RobotCapability(
                name="release",
                parameters=["object_name"],
                description="Release a grasped object"
            ),
            RobotCapability(
                name="navigate_to",
                parameters=["location"],
                description="Navigate to a named location"
            ),
            RobotCapability(
                name="detect_object",
                parameters=["object_name"],
                description="Detect if an object is present in the environment"
            ),
            RobotCapability(
                name="turn",
                parameters=["direction", "angle"],
                description="Turn the robot in a specific direction by a given angle"
            ),
            RobotCapability(
                name="stop",
                parameters=[],
                description="Stop all robot movement"
            ),
            RobotCapability(
                name="wait",
                parameters=["duration"],
                description="Wait for a specified duration in seconds"
            )
        ]
        return capabilities

    def _create_planning_prompt(self, command: str) -> str:
        """
        Create a prompt for the LLM to generate a task plan

        Args:
            command: Natural language command from user

        Returns:
            Formatted prompt string
        """
        capabilities_str = "\n".join([
            f"- {cap.name}({', '.join(cap.parameters)}): {cap.description}"
            for cap in self.robot_capabilities
        ])

        prompt = f"""
You are a helpful assistant that converts natural language commands into executable robot action sequences.
The robot has the following capabilities:

{capabilities_str}

Your task is to decompose the following command into a sequence of actions that the robot can execute.
Return your response as a JSON object with the following structure:
{{
    "command": "original command",
    "actions": [
        {{
            "action": "action_name",
            "parameters": {{"param1": "value1", "param2": "value2"}},
            "description": "Brief description of what this action does"
        }}
    ],
    "confidence": 0.0-1.0 (confidence in the plan)
}}

Command: "{command}"

Response (JSON only, no other text):
"""
        return prompt

    def _parse_llm_response(self, response_text: str) -> Optional[TaskPlan]:
        """
        Parse the LLM response and extract the task plan

        Args:
            response_text: Raw response from LLM

        Returns:
            TaskPlan object or None if parsing fails
        """
        try:
            # Extract JSON from response (in case there's extra text)
            json_match = re.search(r'\{.*\}', response_text, re.DOTALL)
            if json_match:
                json_str = json_match.group(0)
                plan_data = json.loads(json_str)

                # Validate the plan structure
                if 'actions' not in plan_data or 'command' not in plan_data:
                    self.logger.error("Invalid plan structure in LLM response")
                    return None

                # Validate actions
                for action in plan_data['actions']:
                    if 'action' not in action:
                        self.logger.error("Action missing 'action' field")
                        return None

                # Create TaskPlan object
                plan = TaskPlan(
                    command=plan_data['command'],
                    actions=plan_data['actions'],
                    confidence=plan_data.get('confidence', 0.8),  # Default confidence
                    execution_time=len(plan_data['actions']) * 2.0  # Estimate execution time
                )

                return plan
            else:
                self.logger.error("No JSON found in LLM response")
                return None

        except json.JSONDecodeError as e:
            self.logger.error(f"Error parsing LLM response as JSON: {e}")
            self.logger.debug(f"Response text: {response_text}")
            return None
        except Exception as e:
            self.logger.error(f"Unexpected error parsing LLM response: {e}")
            return None

    def plan_task(self, command: str) -> Optional[TaskPlan]:
        """
        Generate a task plan for the given command using LLM

        Args:
            command: Natural language command from user

        Returns:
            TaskPlan object or None if planning fails
        """
        self.logger.info(f"Planning task for command: '{command}'")

        if self.use_mock:
            # Mock implementation for demonstration
            return self._mock_plan_task(command)

        try:
            prompt = self._create_planning_prompt(command)

            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,  # Lower temperature for more consistent outputs
                max_tokens=1000
            )

            response_text = response.choices[0].message.content
            plan = self._parse_llm_response(response_text)

            if plan:
                self.logger.info(f"Generated plan with {len(plan.actions)} actions")
                return plan
            else:
                self.logger.error("Failed to parse LLM response into valid plan")
                return None

        except Exception as e:
            self.logger.error(f"Error in LLM planning: {e}")
            return None

    def _mock_plan_task(self, command: str) -> Optional[TaskPlan]:
        """
        Mock implementation of task planning for demonstration

        Args:
            command: Natural language command from user

        Returns:
            TaskPlan object or None if planning fails
        """
        # Simulate processing delay
        time.sleep(0.5)

        # Simple rule-based mock planning for demonstration
        command_lower = command.lower()

        if "pick" in command_lower or "grasp" in command_lower:
            # Example: "Pick up the red cube from the table"
            actions = [
                {
                    "action": "detect_object",
                    "parameters": {"object_name": "red cube"},
                    "description": "Detect the red cube in the environment"
                },
                {
                    "action": "navigate_to",
                    "parameters": {"location": "table"},
                    "description": "Navigate to the table"
                },
                {
                    "action": "grasp",
                    "parameters": {"object_name": "red cube"},
                    "description": "Grasp the red cube"
                }
            ]
        elif "move" in command_lower or "go to" in command_lower:
            # Example: "Move to the kitchen" or "Go to the table"
            actions = [
                {
                    "action": "navigate_to",
                    "parameters": {"location": "kitchen" if "kitchen" in command_lower else "table"},
                    "description": "Navigate to the specified location"
                }
            ]
        elif "place" in command_lower or "put" in command_lower:
            # Example: "Place the object on the shelf"
            actions = [
                {
                    "action": "navigate_to",
                    "parameters": {"location": "shelf"},
                    "description": "Navigate to the shelf"
                },
                {
                    "action": "release",
                    "parameters": {"object_name": "object"},
                    "description": "Release the object"
                }
            ]
        else:
            # Default action for unrecognized commands
            actions = [
                {
                    "action": "stop",
                    "parameters": {},
                    "description": "Stop robot - command not understood"
                }
            ]

        plan = TaskPlan(
            command=command,
            actions=actions,
            confidence=0.85,  # Mock confidence
            execution_time=len(actions) * 2.0  # Estimate execution time
        )

        self.logger.info(f"Mock plan generated with {len(actions)} actions")
        return plan

    def validate_plan(self, plan: TaskPlan) -> Tuple[bool, List[str]]:
        """
        Validate the generated plan for safety and feasibility

        Args:
            plan: Task plan to validate

        Returns:
            Tuple of (is_valid, list_of_issues)
        """
        issues = []

        # Check if all actions are valid capabilities
        valid_action_names = [cap.name for cap in self.robot_capabilities]
        for action in plan.actions:
            if action['action'] not in valid_action_names:
                issues.append(f"Unknown action: {action['action']}")

        # Check parameter validity
        for action in plan.actions:
            action_name = action['action']
            action_params = action.get('parameters', {})

            capability = next((cap for cap in self.robot_capabilities if cap.name == action_name), None)
            if capability:
                # Check if required parameters are present
                for param in capability.parameters:
                    if param not in action_params:
                        issues.append(f"Missing required parameter '{param}' for action '{action_name}'")

        # Check for potential safety issues
        for i, action in enumerate(plan.actions):
            if action['action'] == 'grasp' and i < len(plan.actions) - 1:
                # Check if there's a release action later
                has_release = any(
                    next_action['action'] == 'release'
                    for next_action in plan.actions[i+1:]
                )
                if not has_release:
                    issues.append("Grasp action without subsequent release action")

        is_valid = len(issues) == 0
        return is_valid, issues


def main():
    """
    Main function to demonstrate the LLM task planner
    """
    print("LLM Task Planner Demo")
    print("="*40)

    # Initialize the planner
    planner = LLMAgent()

    # Test commands
    test_commands = [
        "Pick up the red cube from the table",
        "Move to the kitchen and wait there",
        "Go to the shelf and place the book",
        "Navigate to the charging station",
        "Detect the blue ball and grasp it"
    ]

    for command in test_commands:
        print(f"\nCommand: '{command}'")

        # Generate plan
        plan = planner.plan_task(command)

        if plan:
            print(f"Plan confidence: {plan.confidence:.2f}")
            print(f"Estimated execution time: {plan.execution_time:.1f}s")
            print("Action sequence:")

            for i, action in enumerate(plan.actions):
                params_str = ", ".join([f"{k}={v}" for k, v in action.get('parameters', {}).items()])
                print(f"  {i+1}. {action['action']}({params_str}) - {action.get('description', '')}")

            # Validate plan
            is_valid, issues = planner.validate_plan(plan)
            if is_valid:
                print("  ✓ Plan is valid")
            else:
                print(f"  ⚠ Plan has {len(issues)} issue(s):")
                for issue in issues:
                    print(f"    - {issue}")
        else:
            print("  ✗ Failed to generate plan")

    print("\nDemo completed!")


if __name__ == "__main__":
    main()