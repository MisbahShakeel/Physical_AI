#!/usr/bin/env python3
"""
AI-Driven Decision Maker for Humanoid Robot

This script implements an AI system that combines LLM and VLM capabilities
to make decisions for a humanoid robot. It processes natural language commands,
analyzes visual information, and generates appropriate robot behaviors.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import numpy as np
import json
import time
from enum import Enum


class RobotState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    MANIPULATING = "manipulating"
    PERCEIVING = "perceiving"
    RESPONDING = "responding"


class AIDecisionMaker(Node):
    def __init__(self):
        super().__init__('ai_decision_maker')

        # Create CvBridge instance
        self.bridge = CvBridge()

        # Subscribe to various inputs
        self.command_subscription = self.create_subscription(
            String,
            '/robot/command',
            self.command_callback,
            10
        )

        self.image_subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/humanoid/laser_scan',
            self.lidar_callback,
            10
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.response_publisher = self.create_publisher(String, '/robot/response', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

        # Timer for decision loop
        self.timer = self.create_timer(0.5, self.decision_loop)  # 2 Hz

        # Robot state and data
        self.current_state = RobotState.IDLE
        self.latest_command = None
        self.command_timestamp = None
        self.latest_image = None
        self.latest_lidar = None
        self.perception_results = {}
        self.pending_action = None

        # Simulated world model
        self.world_objects = {
            'red_box': {'position': (2.0, 1.0, 0.0), 'type': 'obstacle'},
            'blue_cylinder': {'position': (-1.0, -1.5, 0.0), 'type': 'target'},
            'green_sphere': {'position': (0.5, 2.0, 0.0), 'type': 'landmark'}
        }

        # Action queue for complex tasks
        self.action_queue = []

        self.get_logger().info('AI Decision Maker initialized')

    def command_callback(self, msg):
        """Process high-level commands from user"""
        command_text = msg.data.lower()
        self.get_logger().info(f'Received command: {command_text}')

        self.latest_command = command_text
        self.command_timestamp = time.time()
        self.current_state = RobotState.RESPONDING

        # Process the command using AI reasoning
        self.process_command(command_text)

    def image_callback(self, msg):
        """Process visual input"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().debug('Received image for processing')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def lidar_callback(self, msg):
        """Process LiDAR input for navigation"""
        self.latest_lidar = msg
        self.get_logger().debug('Received LiDAR data')

    def process_command(self, command_text):
        """AI reasoning to interpret command and plan actions"""
        # Simulate AI processing - in real implementation this would use LLM
        self.get_logger().info(f'Processing command with AI reasoning: {command_text}')

        # Simple command interpretation for demonstration
        if 'go to' in command_text or 'navigate to' in command_text:
            self.handle_navigation_command(command_text)
        elif 'find' in command_text or 'look for' in command_text:
            self.handle_perception_command(command_text)
        elif 'move' in command_text or 'go' in command_text:
            self.handle_movement_command(command_text)
        else:
            # Default response
            response = f"I understand you said '{command_text}'. I'm ready to help."
            self.publish_response(response)

    def handle_navigation_command(self, command_text):
        """Handle navigation-related commands"""
        # Extract target from command (simplified)
        target = None
        for obj_name in self.world_objects.keys():
            if obj_name.replace('_', ' ') in command_text:
                target = obj_name
                break

        if target and target in self.world_objects:
            target_pos = self.world_objects[target]['position']
            self.get_logger().info(f'Navigating to {target} at {target_pos}')

            # Plan path to target
            self.plan_navigation_to_target(target_pos)

            response = f"I'm navigating to the {target.replace('_', ' ')}."
            self.publish_response(response)
        else:
            response = f"I couldn't identify the target in your command: {command_text}"
            self.publish_response(response)

    def handle_perception_command(self, command_text):
        """Handle perception-related commands"""
        self.current_state = RobotState.PERCEIVING

        # Analyze current scene
        scene_description = self.analyze_scene()

        # Formulate response based on scene and command
        response = f"I analyzed the scene and {scene_description}"
        self.publish_response(response)

    def handle_movement_command(self, command_text):
        """Handle simple movement commands"""
        # Simple movement commands
        if 'forward' in command_text:
            self.move_forward()
        elif 'backward' in command_text:
            self.move_backward()
        elif 'left' in command_text:
            self.turn_left()
        elif 'right' in command_text:
            self.turn_right()
        else:
            # Use AI to determine appropriate action
            self.use_ai_for_movement(command_text)

    def use_ai_for_movement(self, command_text):
        """Use AI reasoning to determine movement from natural language"""
        # Simulate AI decision making
        # In a real implementation, this would use an LLM to interpret the command
        if any(word in command_text for word in ['approach', 'toward', 'move to']):
            response = "I'm determining the best approach based on your command."
            self.publish_response(response)
            # This would trigger more complex planning
        else:
            response = f"I'm interpreting your command '{command_text}' and planning appropriate movement."
            self.publish_response(response)

    def plan_navigation_to_target(self, target_pos):
        """Plan navigation to a specific target position"""
        self.get_logger().info(f'Planning navigation to target: {target_pos}')

        # Simple navigation planning (in real implementation, this would be more sophisticated)
        cmd = Twist()
        cmd.linear.x = 0.3  # Move forward initially
        cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd)
        self.current_state = RobotState.NAVIGATING

    def analyze_scene(self):
        """Analyze current scene using visual and spatial information"""
        if self.latest_image is not None:
            # Simulate scene analysis
            # In real implementation, this would use computer vision/VLM
            object_count = len(self.world_objects)
            landmark_count = sum(1 for obj in self.world_objects.values() if obj['type'] == 'landmark')

            return f"found {object_count} objects including {landmark_count} landmarks."
        else:
            return "don't have current visual information to analyze."

    def move_forward(self):
        """Execute forward movement"""
        cmd = Twist()
        cmd.linear.x = 0.5
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Moving forward')
        self.publish_response("Moving forward as requested.")

    def move_backward(self):
        """Execute backward movement"""
        cmd = Twist()
        cmd.linear.x = -0.5
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Moving backward')
        self.publish_response("Moving backward as requested.")

    def turn_left(self):
        """Execute left turn"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Turning left')
        self.publish_response("Turning left as requested.")

    def turn_right(self):
        """Execute right turn"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -0.5
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Turning right')
        self.publish_response("Turning right as requested.")

    def publish_response(self, response_text):
        """Publish AI response to user"""
        response_msg = String()
        response_msg.data = response_text
        self.response_publisher.publish(response_msg)
        self.get_logger().info(f'AI Response: {response_text}')

    def decision_loop(self):
        """Main AI decision-making loop"""
        if self.current_state == RobotState.NAVIGATING:
            # Check navigation progress and update as needed
            self.check_navigation_status()
        elif self.current_state == RobotState.PERCEIVING:
            # Process perception data if available
            if self.latest_image is not None:
                # Update perception results
                self.perception_results = self.analyze_scene()
                self.current_state = RobotState.IDLE
        elif self.current_state == RobotState.RESPONDING:
            # Command was processed, return to idle
            self.current_state = RobotState.IDLE

    def check_navigation_status(self):
        """Check if navigation is complete or needs adjustment"""
        # In a real implementation, this would check actual robot position vs target
        # For simulation, we'll just continue moving
        pass


def main(args=None):
    rclpy.init(args=args)

    ai_node = AIDecisionMaker()

    try:
        rclpy.spin(ai_node)
    except KeyboardInterrupt:
        ai_node.get_logger().info('Shutting down AI Decision Maker')
    finally:
        # Stop the robot before shutting down
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        ai_node.cmd_vel_publisher.publish(cmd)
        ai_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()