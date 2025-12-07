#!/usr/bin/env python3
"""
LLM Pick and Place Controller

This script implements a system where an LLM plans a multi-step pick and place
task for a simulated humanoid robot, demonstrating cognitive planning and action sequencing.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import json
import time
from typing import Dict, List, Any, Optional
import logging
import threading
from dataclasses import dataclass


@dataclass
class ObjectInfo:
    """Information about an object in the environment"""
    name: str
    position: Point
    grasped: bool = False


@dataclass
class LocationInfo:
    """Information about a location in the environment"""
    name: str
    position: Point
    description: str


class LLMPickPlaceController(Node):
    """
    Main controller for LLM-based pick and place operations
    """

    def __init__(self):
        super().__init__('llm_pick_place_controller')

        # Environment information
        self.objects = {}
        self.locations = {}
        self.robot_pose = Point(x=0.0, y=0.0, z=0.0)
        self.robot_grasping = False

        # Initialize environment
        self._initialize_environment()

        # LLM planner (using mock implementation for demo)
        self.llm_planner = MockLLMPlanner()

        # Execution state
        self.current_plan = []
        self.plan_index = 0
        self.is_executing = False
        self.execution_thread = None

        # Publishers
        self.status_publisher = self.create_publisher(
            String,
            '/pick_place_status',
            10
        )

        self.feedback_publisher = self.create_publisher(
            String,
            '/pick_place_feedback',
            10
        )

        # Subscribers
        self.command_subscription = self.create_subscription(
            String,
            '/pick_place_command',
            self.command_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Timer for periodic updates
        self.update_timer = self.create_timer(1.0, self.update_callback)

        # Action execution parameters
        self.navigation_speed = 0.5
        self.manipulation_speed = 0.1
        self.action_timeout = 10.0

        self.get_logger().info('LLM Pick and Place Controller initialized')

    def _initialize_environment(self):
        """Initialize the simulated environment with objects and locations"""
        # Define locations
        self.locations = {
            'table': LocationInfo('table', Point(x=1.0, y=0.0, z=0.0), 'Main work table'),
            'shelf': LocationInfo('shelf', Point(x=2.0, y=1.0, z=0.0), 'Storage shelf'),
            'desk': LocationInfo('desk', Point(x=-1.0, y=1.0, z=0.0), 'Work desk'),
            'charger': LocationInfo('charger', Point(x=0.0, y=-2.0, z=0.0), 'Charging station')
        }

        # Define objects
        self.objects = {
            'red_cube': ObjectInfo('red_cube', Point(x=1.0, y=0.1, z=0.0)),
            'blue_sphere': ObjectInfo('blue_sphere', Point(x=1.2, y=0.1, z=0.0)),
            'green_cylinder': ObjectInfo('green_cylinder', Point(x=1.4, y=0.1, z=0.0)),
            'book': ObjectInfo('book', Point(x=2.0, y=1.1, z=0.0))
        }

        self.get_logger().info(f'Initialized environment with {len(self.objects)} objects and {len(self.locations)} locations')

    def command_callback(self, msg: String):
        """Handle incoming pick and place commands"""
        command = msg.data
        self.get_logger().info(f"Received command: '{command}'")

        # Generate plan using LLM
        plan = self.llm_planner.plan_task(command)

        if plan:
            self.get_logger().info(f"Generated plan with {len(plan.actions)} actions")
            self.execute_plan(plan.actions)
        else:
            self.get_logger().error("Failed to generate plan for command")
            feedback_msg = String()
            feedback_msg.data = f"Failed to understand command: {command}"
            self.feedback_publisher.publish(feedback_msg)

    def odom_callback(self, msg: Odometry):
        """Update robot pose from odometry"""
        self.robot_pose = msg.pose.pose.position

    def update_callback(self):
        """Periodic update callback"""
        status_msg = String()
        status_msg.data = f"Objects: {len(self.objects)}, Locations: {len(self.locations)}, Executing: {self.is_executing}"
        self.status_publisher.publish(status_msg)

    def execute_plan(self, plan: List[Dict[str, Any]]):
        """Execute the generated plan in a separate thread"""
        if self.is_executing:
            self.get_logger().warn("Plan execution already in progress, cancelling current plan")
            self.cancel_execution()

        self.current_plan = plan
        self.plan_index = 0
        self.is_executing = True

        # Start execution in a separate thread
        self.execution_thread = threading.Thread(target=self._execute_plan_thread)
        self.execution_thread.start()

    def _execute_plan_thread(self):
        """Execute the plan in a separate thread"""
        try:
            while self.plan_index < len(self.current_plan) and self.is_executing:
                action = self.current_plan[self.plan_index]
                self.get_logger().info(f"Executing action: {action['action']}")

                success = self._execute_action(action)

                if success:
                    # Move to next action
                    self.plan_index += 1
                    feedback_msg = String()
                    feedback_msg.data = f"Completed: {action['action']}"
                    self.feedback_publisher.publish(feedback_msg)
                else:
                    self.get_logger().error(f"Action failed: {action['action']}")
                    self.is_executing = False
                    break

            if self.is_executing and self.plan_index >= len(self.current_plan):
                # Plan completed successfully
                self.get_logger().info("Plan completed successfully")
                feedback_msg = String()
                feedback_msg.data = "Pick and place task completed successfully"
                self.feedback_publisher.publish(feedback_msg)
            else:
                feedback_msg = String()
                feedback_msg.data = "Pick and place task failed or was cancelled"
                self.feedback_publisher.publish(feedback_msg)

        except Exception as e:
            self.get_logger().error(f"Error in plan execution thread: {e}")
        finally:
            self.is_executing = False

    def _execute_action(self, action: Dict[str, Any]) -> bool:
        """Execute a single action"""
        action_name = action['action']
        parameters = action.get('parameters', {})

        try:
            if action_name == 'navigate_to':
                return self._execute_navigate_to(parameters)
            elif action_name == 'detect_object':
                return self._execute_detect_object(parameters)
            elif action_name == 'grasp':
                return self._execute_grasp(parameters)
            elif action_name == 'release':
                return self._execute_release(parameters)
            elif action_name == 'move_to':
                return self._execute_move_to(parameters)
            elif action_name == 'wait':
                return self._execute_wait(parameters)
            else:
                self.get_logger().warn(f"Unknown action: {action_name}")
                return False

        except Exception as e:
            self.get_logger().error(f"Error executing action {action_name}: {e}")
            return False

    def _execute_navigate_to(self, parameters: Dict[str, Any]) -> bool:
        """Execute navigate_to action"""
        location_name = parameters.get('location', '')

        if location_name in self.locations:
            target_location = self.locations[location_name]
            self.get_logger().info(f"Navigating to {location_name} at ({target_location.position.x}, {target_location.position.y})")

            # Simulate navigation
            time.sleep(2.0)  # Simulate navigation time

            # Update robot position
            self.robot_pose = target_location.position

            self.get_logger().info(f"Reached {location_name}")
            return True
        else:
            self.get_logger().error(f"Unknown location: {location_name}")
            return False

    def _execute_detect_object(self, parameters: Dict[str, Any]) -> bool:
        """Execute detect_object action"""
        object_name = parameters.get('object_name', '')

        if object_name in self.objects:
            obj = self.objects[object_name]
            self.get_logger().info(f"Detected {object_name} at ({obj.position.x}, {obj.position.y})")

            # Simulate detection
            time.sleep(0.5)

            return True
        else:
            self.get_logger().error(f"Object not found: {object_name}")
            return False

    def _execute_grasp(self, parameters: Dict[str, Any]) -> bool:
        """Execute grasp action"""
        object_name = parameters.get('object_name', '')

        if object_name in self.objects:
            obj = self.objects[object_name]
            if not obj.grasped:
                obj.grasped = True
                self.robot_grasping = True
                self.get_logger().info(f"Grasped {object_name}")

                # Simulate grasping
                time.sleep(1.0)

                return True
            else:
                self.get_logger().warn(f"{object_name} is already grasped")
                return True  # Consider this a success
        else:
            self.get_logger().error(f"Cannot grasp unknown object: {object_name}")
            return False

    def _execute_release(self, parameters: Dict[str, Any]) -> bool:
        """Execute release action"""
        object_name = parameters.get('object_name', '')

        if object_name in self.objects:
            obj = self.objects[object_name]
            if obj.grasped:
                obj.grasped = False
                self.robot_grasping = False
                self.get_logger().info(f"Released {object_name}")

                # Simulate releasing
                time.sleep(1.0)

                return True
            else:
                self.get_logger().warn(f"{object_name} is not currently grasped")
                return True  # Consider this a success
        else:
            self.get_logger().error(f"Cannot release unknown object: {object_name}")
            return False

    def _execute_move_to(self, parameters: Dict[str, Any]) -> bool:
        """Execute move_to action"""
        x = parameters.get('x', self.robot_pose.x)
        y = parameters.get('y', self.robot_pose.y)
        z = parameters.get('z', self.robot_pose.z)

        self.get_logger().info(f"Moving to ({x}, {y}, {z})")

        # Simulate movement
        time.sleep(1.0)

        # Update robot position
        self.robot_pose.x = x
        self.robot_pose.y = y
        self.robot_pose.z = z

        return True

    def _execute_wait(self, parameters: Dict[str, Any]) -> bool:
        """Execute wait action"""
        duration = parameters.get('duration', 1.0)

        self.get_logger().info(f"Waiting for {duration} seconds")
        time.sleep(duration)

        return True

    def cancel_execution(self):
        """Cancel current plan execution"""
        self.is_executing = False
        if self.execution_thread:
            self.execution_thread.join(timeout=1.0)

        self.get_logger().info("Plan execution cancelled")

        feedback_msg = String()
        feedback_msg.data = "Pick and place task cancelled"
        self.feedback_publisher.publish(feedback_msg)

    def get_environment_state(self) -> Dict[str, Any]:
        """Get the current state of the environment"""
        return {
            'objects': {name: {'position': {'x': obj.position.x, 'y': obj.position.y, 'z': obj.position.z},
                              'grasped': obj.grasped}
                       for name, obj in self.objects.items()},
            'locations': {name: {'position': {'x': loc.position.x, 'y': loc.position.y, 'z': loc.position.z}}
                         for name, loc in self.locations.items()},
            'robot': {'position': {'x': self.robot_pose.x, 'y': self.robot_pose.y, 'z': self.robot_pose.z},
                     'grasping': self.robot_grasping}
        }


class MockLLMPlanner:
    """
    Mock implementation of LLM planner for demonstration
    In a real implementation, this would interface with an actual LLM
    """

    def __init__(self):
        self.command_patterns = {
            'pick_place': [
                r'pick.*place',
                r'move.*from.*to',
                r'get.*and.*put',
            ],
            'pick_only': [
                r'pick.*up',
                r'grasp.*',
                r'take.*',
            ],
            'place_only': [
                r'place.*',
                r'put.*',
                r'release.*',
            ]
        }

    def plan_task(self, command: str) -> Optional[Any]:
        """Generate a plan for the given command"""
        import re

        # Simulate processing delay
        time.sleep(0.5)

        # Simple rule-based planning for demonstration
        command_lower = command.lower()

        if any(re.search(pattern, command_lower) for pattern in self.command_patterns['pick_place']):
            # Example: "Pick up the red cube and place it on the table"
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
                },
                {
                    "action": "navigate_to",
                    "parameters": {"location": "shelf"},
                    "description": "Navigate to the shelf"
                },
                {
                    "action": "release",
                    "parameters": {"object_name": "red cube"},
                    "description": "Release the red cube"
                }
            ]
        elif any(re.search(pattern, command_lower) for pattern in self.command_patterns['pick_only']):
            # Example: "Pick up the book"
            actions = [
                {
                    "action": "detect_object",
                    "parameters": {"object_name": "book"},
                    "description": "Detect the book in the environment"
                },
                {
                    "action": "navigate_to",
                    "parameters": {"location": "shelf"},
                    "description": "Navigate to the shelf"
                },
                {
                    "action": "grasp",
                    "parameters": {"object_name": "book"},
                    "description": "Grasp the book"
                }
            ]
        elif any(re.search(pattern, command_lower) for pattern in self.command_patterns['place_only']):
            # Example: "Place the object on the desk"
            actions = [
                {
                    "action": "navigate_to",
                    "parameters": {"location": "desk"},
                    "description": "Navigate to the desk"
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
                    "action": "wait",
                    "parameters": {"duration": 2.0},
                    "description": "Wait - command not understood"
                }
            ]

        # Create a mock plan object
        class MockPlan:
            def __init__(self, actions):
                self.actions = actions

        plan = MockPlan(actions)
        return plan


def main(args=None):
    """Main function to run the LLM pick and place controller"""
    rclpy.init(args=args)

    controller = LLMPickPlaceController()

    try:
        controller.get_logger().info("LLM Pick and Place Controller starting...")
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down LLM Pick and Place Controller")
        controller.cancel_execution()
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()