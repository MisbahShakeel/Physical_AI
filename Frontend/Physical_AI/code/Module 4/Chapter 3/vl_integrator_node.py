#!/usr/bin/env python3
"""
Vision-Language Integrator Node for VLA Control Loop

This script implements a ROS 2 node that integrates visual perception,
voice commands, and LLM planning for Vision-Language-Action control.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import json
import time
from typing import Dict, List, Any, Optional
import logging
import threading
from dataclasses import dataclass


@dataclass
class VLAState:
    """Represents the current state of the VLA system"""
    vision_input: Optional[Dict] = None
    language_input: Optional[str] = None
    llm_plan: Optional[List[Dict]] = None
    robot_pose: Optional[Pose] = None
    detected_objects: List[Dict] = None
    execution_status: str = "idle"
    timestamp: float = 0.0


class VLIntegratorNode(Node):
    """
    ROS 2 node that integrates vision, language, and action for VLA control
    """

    def __init__(self):
        super().__init__('vl_integrator_node')

        # Initialize components (using mock implementations for demo)
        self.vision_processor = MockVisionProcessor()
        self.language_processor = MockLanguageProcessor()
        self.llm_planner = MockLLMPlanner()

        # Internal state
        self.vla_state = VLAState()
        self.execution_thread = None
        self.is_executing = False
        self.plan_queue = []

        # Publishers
        self.action_publisher = self.create_publisher(
            String,
            '/vla/actions',
            10
        )

        self.status_publisher = self.create_publisher(
            String,
            '/vla/status',
            10
        )

        self.feedback_publisher = self.create_publisher(
            String,
            '/vla/feedback',
            10
        )

        # Subscribers
        self.vision_subscription = self.create_subscription(
            Detection2DArray,
            '/vla/detections',
            self.vision_callback,
            10
        )

        self.language_subscription = self.create_subscription(
            String,
            '/voice_commands',
            self.language_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Timer for VLA processing
        self.vla_timer = self.create_timer(
            0.2,  # 5 Hz
            self.vla_processing_callback
        )

        # VLA parameters
        self.fusion_confidence_threshold = 0.7
        self.max_plan_queue_size = 5
        self.action_execution_timeout = 10.0

        self.get_logger().info('VLA Integrator node initialized')

    def vision_callback(self, msg: Detection2DArray):
        """
        Handle incoming vision detections
        """
        try:
            # Convert detections to internal format
            objects = []
            for detection in msg.detections:
                if detection.results:
                    obj = {
                        'name': detection.results[0].hypothesis.class_id,
                        'confidence': detection.results[0].hypothesis.score,
                        'bbox': {
                            'center_x': detection.bbox.center.x,
                            'center_y': detection.bbox.center.y,
                            'size_x': detection.bbox.size_x,
                            'size_y': detection.bbox.size_y
                        }
                    }
                    objects.append(obj)

            self.vla_state.vision_input = {
                'objects': objects,
                'timestamp': time.time(),
                'frame_id': msg.header.frame_id
            }
            self.vla_state.detected_objects = objects

            self.get_logger().debug(f"Received vision input with {len(objects)} objects")

        except Exception as e:
            self.get_logger().error(f"Error processing vision message: {e}")

    def language_callback(self, msg: String):
        """
        Handle incoming language commands
        """
        command = msg.data.strip()
        if command:
            self.vla_state.language_input = command
            self.get_logger().info(f"Received language command: '{command}'")

    def odom_callback(self, msg: Odometry):
        """
        Update robot pose from odometry
        """
        self.vla_state.robot_pose = msg.pose.pose

    def vla_processing_callback(self):
        """
        Main VLA processing callback
        """
        # Check if we have both vision and language inputs to generate a plan
        if (self.vla_state.language_input and
            self.vla_state.detected_objects is not None and
            self.vla_state.execution_status == "idle"):

            # Generate plan using LLM
            plan = self.llm_planner.plan_task(
                self.vla_state.language_input,
                self.vla_state.detected_objects
            )

            if plan:
                self.get_logger().info(f"Generated plan with {len(plan)} actions")
                self.vla_state.llm_plan = plan
                self.vla_state.execution_status = "planned"

                # Execute the plan
                self.execute_plan(plan)

                # Clear the language input after processing
                self.vla_state.language_input = None

        # Update status
        status_msg = String()
        status_msg.data = f"VLA State: {self.vla_state.execution_status}, Objects: {len(self.vla_state.detected_objects or [])}"
        self.status_publisher.publish(status_msg)

    def execute_plan(self, plan: List[Dict[str, Any]]):
        """
        Execute the generated plan in a separate thread
        """
        if self.is_executing:
            self.get_logger().warn("Plan execution already in progress, queuing new plan")
            if len(self.plan_queue) < self.max_plan_queue_size:
                self.plan_queue.append(plan)
            else:
                self.get_logger().warn("Plan queue full, discarding new plan")
            return

        self.is_executing = True
        self.vla_state.execution_status = "executing"

        # Start execution in a separate thread
        self.execution_thread = threading.Thread(
            target=self._execute_plan_thread,
            args=(plan,)
        )
        self.execution_thread.start()

    def _execute_plan_thread(self, plan: List[Dict[str, Any]]):
        """
        Execute the plan in a separate thread
        """
        try:
            for i, action in enumerate(plan):
                if not self.is_executing:  # Check for cancellation
                    self.get_logger().info("Plan execution cancelled")
                    break

                self.get_logger().info(f"Executing action {i+1}/{len(plan)}: {action['action']}")

                # Publish action to be executed
                action_msg = String()
                action_msg.data = json.dumps(action)
                self.action_publisher.publish(action_msg)

                # Simulate action execution time
                time.sleep(1.0)

                # Publish feedback
                feedback_msg = String()
                feedback_msg.data = f"Completed: {action['action']}"
                self.feedback_publisher.publish(feedback_msg)

            if self.is_executing:  # Only if not cancelled
                self.get_logger().info("Plan execution completed")
                self.vla_state.execution_status = "completed"

                # Publish completion feedback
                feedback_msg = String()
                feedback_msg.data = "VLA task completed successfully"
                self.feedback_publisher.publish(feedback_msg)

                # Process next plan in queue if available
                if self.plan_queue:
                    next_plan = self.plan_queue.pop(0)
                    self.execute_plan(next_plan)

        except Exception as e:
            self.get_logger().error(f"Error in plan execution: {e}")
            self.vla_state.execution_status = "failed"

            feedback_msg = String()
            feedback_msg.data = f"VLA task failed: {str(e)}"
            self.feedback_publisher.publish(feedback_msg)

        finally:
            self.is_executing = False
            if not self.plan_queue:  # Only reset status if no plans queued
                self.vla_state.execution_status = "idle"

    def cancel_execution(self):
        """
        Cancel current plan execution
        """
        self.is_executing = False
        self.vla_state.execution_status = "cancelled"

        if self.execution_thread:
            self.execution_thread.join(timeout=1.0)

        self.get_logger().info("VLA execution cancelled")

        feedback_msg = String()
        feedback_msg.data = "VLA task cancelled"
        self.feedback_publisher.publish(feedback_msg)

    def get_vla_state(self) -> Dict[str, Any]:
        """
        Get the current state of the VLA system
        """
        return {
            'vision_input': self.vla_state.vision_input,
            'language_input': self.vla_state.language_input,
            'llm_plan': self.vla_state.llm_plan,
            'robot_pose': self.vla_state.robot_pose,
            'detected_objects': self.vla_state.detected_objects,
            'execution_status': self.vla_state.execution_status,
            'timestamp': self.vla_state.timestamp
        }

    def request_vla_state(self) -> String:
        """
        Request current VLA state as a JSON string
        """
        state = self.get_vla_state()
        state_msg = String()
        state_msg.data = json.dumps(state, indent=2)
        return state_msg


class MockVisionProcessor:
    """
    Mock vision processor for demonstration
    In a real implementation, this would process actual visual data
    """

    def __init__(self):
        pass

    def process_image(self, image_data):
        """
        Process image and return object detections
        """
        # In a real implementation, this would run object detection
        # For demo, return mock results
        return [
            {"name": "red_cube", "confidence": 0.85, "position": [1.0, 0.5, 0.0]},
            {"name": "blue_sphere", "confidence": 0.78, "position": [1.5, 0.2, 0.0]}
        ]


class MockLanguageProcessor:
    """
    Mock language processor for demonstration
    In a real implementation, this would process natural language
    """

    def __init__(self):
        pass

    def process_command(self, command: str) -> Dict[str, Any]:
        """
        Process natural language command and return structured intent
        """
        # Simple keyword-based parsing for demo
        command_lower = command.lower()

        intent = {
            "action": "unknown",
            "target_object": None,
            "target_location": None,
            "confidence": 0.8
        }

        if "pick" in command_lower or "grasp" in command_lower:
            intent["action"] = "pick"
        elif "place" in command_lower or "put" in command_lower:
            intent["action"] = "place"
        elif "move" in command_lower or "go" in command_lower:
            intent["action"] = "navigate"

        # Extract object if mentioned
        objects = ["cube", "sphere", "cylinder", "book", "bottle"]
        for obj in objects:
            if obj in command_lower:
                intent["target_object"] = obj
                break

        # Extract location if mentioned
        locations = ["table", "shelf", "desk", "kitchen"]
        for loc in locations:
            if loc in command_lower:
                intent["target_location"] = loc
                break

        return intent


class MockLLMPlanner:
    """
    Mock LLM planner for demonstration
    In a real implementation, this would interface with an actual LLM
    """

    def __init__(self):
        pass

    def plan_task(self, command: str, detected_objects: List[Dict[str, Any]]) -> Optional[List[Dict[str, Any]]]:
        """
        Generate a plan based on command and detected objects
        """
        # Simulate processing delay
        time.sleep(0.2)

        # Simple rule-based planning for demonstration
        command_lower = command.lower()

        if "pick" in command_lower or "grasp" in command_lower:
            # Find the target object among detected objects
            target_obj = None
            for obj in detected_objects:
                if obj['name'] in command_lower:
                    target_obj = obj['name']
                    break

            if target_obj:
                plan = [
                    {
                        "action": "navigate_to_object",
                        "parameters": {"object_name": target_obj},
                        "description": f"Navigate to {target_obj}"
                    },
                    {
                        "action": "grasp_object",
                        "parameters": {"object_name": target_obj},
                        "description": f"Grasp {target_obj}"
                    }
                ]
            else:
                # If object not found, ask for clarification
                plan = [
                    {
                        "action": "request_clarification",
                        "parameters": {"message": f"Object not found: {command}"},
                        "description": "Request clarification from user"
                    }
                ]

        elif "move" in command_lower or "go to" in command_lower:
            plan = [
                {
                    "action": "navigate_to_location",
                    "parameters": {"location": "target_location"},
                    "description": "Navigate to target location"
                }
            ]

        elif "find" in command_lower:
            plan = [
                {
                    "action": "scan_environment",
                    "parameters": {},
                    "description": "Scan environment for requested object"
                }
            ]

        else:
            # Default action for unrecognized commands
            plan = [
                {
                    "action": "wait",
                    "parameters": {"duration": 2.0},
                    "description": "Wait - command not understood"
                }
            ]

        return plan


def main(args=None):
    """
    Main function to run the VLA integrator node
    """
    rclpy.init(args=args)

    node = VLIntegratorNode()

    try:
        node.get_logger().info("VLA Integrator node starting...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down VLA Integrator node")
        node.cancel_execution()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()