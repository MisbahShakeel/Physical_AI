#!/usr/bin/env python3
"""
Voice Command Robot Controller

This script implements a voice command system for a simulated humanoid robot,
converting speech to simple actions like "move forward", "turn left", etc.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np
import time
import re
from typing import Dict, List, Optional, Tuple
import logging


class VoiceRobotController(Node):
    """
    Main controller for voice command robot system
    """

    def __init__(self):
        super().__init__('voice_robot_controller')

        # Command vocabulary and mapping
        self.command_mapping = {
            # Movement commands
            r'move forward|go forward|forward': self.move_forward,
            r'move backward|go backward|backward|back': self.move_backward,
            r'turn left|left turn|turn to the left': self.turn_left,
            r'turn right|right turn|turn to the right': self.turn_right,
            r'move left|strafe left': self.move_left,
            r'move right|strafe right': self.move_right,
            r'stop|halt|freeze': self.stop_robot,
            r'go|start|continue': self.resume_robot,

            # Action commands
            r'dance|dance for me': self.perform_dance,
            r'wave|wave hello|wave goodbye': self.wave_hand,
            r'raise your hand|lift your hand': self.raise_hand,
            r'lower your hand|put down your hand': self.lower_hand,
            r'hello|hi|greetings': self.greet_user,
            r'goodbye|bye|see you': self.say_goodbye,
            r'help|what can you do': self.provide_help,
            r'listen|ready to listen': self.activate_voice_control,
            r'sleep|stop listening': self.deactivate_voice_control,
        }

        # Robot state
        self.robot_active = True
        self.voice_control_active = True
        self.last_command_time = time.time()
        self.command_history = []

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.status_publisher = self.create_publisher(
            String,
            '/robot_status',
            10
        )

        self.voice_feedback_publisher = self.create_publisher(
            String,
            '/voice_feedback',
            10
        )

        # Subscribers
        self.voice_command_subscription = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10
        )

        self.voice_control_subscription = self.create_subscription(
            String,
            '/voice_control',
            self.voice_control_callback,
            10
        )

        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.status_callback)

        # Robot movement parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        self.action_duration = 2.0  # seconds for actions

        # Set up logging
        self.get_logger().info('Voice Robot Controller initialized')
        self.publish_status("VOICE_ROBOT_CONTROLLER_READY")

    def voice_command_callback(self, msg: String):
        """
        Process incoming voice commands
        """
        if not self.voice_control_active:
            return

        command_text = msg.data.lower().strip()
        self.get_logger().info(f"Received voice command: '{command_text}'")

        # Record command in history
        self.command_history.append({
            'command': command_text,
            'timestamp': time.time()
        })

        # Process the command
        action_taken = self.process_command(command_text)

        if action_taken:
            self.last_command_time = time.time()
            feedback_msg = String()
            feedback_msg.data = f"Executed: {command_text}"
            self.voice_feedback_publisher.publish(feedback_msg)
        else:
            # Command not recognized
            feedback_msg = String()
            feedback_msg.data = f"Command not recognized: {command_text}"
            self.voice_feedback_publisher.publish(feedback_msg)
            self.get_logger().warn(f"Unrecognized command: {command_text}")

    def voice_control_callback(self, msg: String):
        """
        Handle voice control commands
        """
        command = msg.data.lower()

        if command == "activate":
            self.activate_voice_control()
        elif command == "deactivate":
            self.deactivate_voice_control()
        elif command == "status":
            self.publish_status("VOICE_CONTROL_STATUS")

    def process_command(self, command: str) -> bool:
        """
        Process a voice command and execute corresponding action

        Args:
            command: The voice command string

        Returns:
            True if command was recognized and executed, False otherwise
        """
        # Check each command pattern
        for pattern, action_func in self.command_mapping.items():
            if re.search(pattern, command):
                try:
                    self.get_logger().info(f"Executing command: {pattern}")
                    action_func()
                    return True
                except Exception as e:
                    self.get_logger().error(f"Error executing command {pattern}: {e}")
                    return False

        # If no pattern matches, command was not recognized
        return False

    def move_forward(self):
        """Move robot forward"""
        self.get_logger().info("Moving forward")
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        time.sleep(self.action_duration)
        self.stop_robot()

    def move_backward(self):
        """Move robot backward"""
        self.get_logger().info("Moving backward")
        cmd = Twist()
        cmd.linear.x = -self.linear_speed
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        time.sleep(self.action_duration)
        self.stop_robot()

    def turn_left(self):
        """Turn robot left"""
        self.get_logger().info("Turning left")
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self.angular_speed
        self.cmd_vel_publisher.publish(cmd)
        time.sleep(self.action_duration)
        self.stop_robot()

    def turn_right(self):
        """Turn robot right"""
        self.get_logger().info("Turning right")
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -self.angular_speed
        self.cmd_vel_publisher.publish(cmd)
        time.sleep(self.action_duration)
        self.stop_robot()

    def move_left(self):
        """Move robot left (strafe)"""
        self.get_logger().info("Moving left")
        cmd = Twist()
        cmd.linear.y = self.linear_speed
        self.cmd_vel_publisher.publish(cmd)
        time.sleep(self.action_duration)
        self.stop_robot()

    def move_right(self):
        """Move robot right (strafe)"""
        self.get_logger().info("Moving right")
        cmd = Twist()
        cmd.linear.y = -self.linear_speed
        self.cmd_vel_publisher.publish(cmd)
        time.sleep(self.action_duration)
        self.stop_robot()

    def stop_robot(self):
        """Stop robot movement"""
        self.get_logger().info("Stopping robot")
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

    def resume_robot(self):
        """Resume robot (placeholder for future use)"""
        self.get_logger().info("Resuming robot operation")
        # Currently just logs the command

    def perform_dance(self):
        """Perform a simple dance routine"""
        self.get_logger().info("Performing dance routine")
        # Simple dance: turn left, turn right, wiggle
        for _ in range(2):
            cmd = Twist()
            cmd.angular.z = self.angular_speed
            self.cmd_vel_publisher.publish(cmd)
            time.sleep(0.5)

            cmd.angular.z = -self.angular_speed
            self.cmd_vel_publisher.publish(cmd)
            time.sleep(0.5)

        self.stop_robot()

    def wave_hand(self):
        """Wave hand action (simulated)"""
        self.get_logger().info("Waving hand")
        # In a real robot, this would control arm joints
        # For simulation, just log the action

    def raise_hand(self):
        """Raise hand action (simulated)"""
        self.get_logger().info("Raising hand")
        # In a real robot, this would control arm joints
        # For simulation, just log the action

    def lower_hand(self):
        """Lower hand action (simulated)"""
        self.get_logger().info("Lowering hand")
        # In a real robot, this would control arm joints
        # For simulation, just log the action

    def greet_user(self):
        """Greet the user"""
        self.get_logger().info("Greeting user")
        feedback_msg = String()
        feedback_msg.data = "Hello! I'm ready to help you."
        self.voice_feedback_publisher.publish(feedback_msg)

    def say_goodbye(self):
        """Say goodbye to the user"""
        self.get_logger().info("Saying goodbye")
        feedback_msg = String()
        feedback_msg.data = "Goodbye! It was nice interacting with you."
        self.voice_feedback_publisher.publish(feedback_msg)

    def provide_help(self):
        """Provide help information"""
        self.get_logger().info("Providing help")
        help_text = "I can understand commands like: move forward, turn left, dance, wave, stop, etc."
        feedback_msg = String()
        feedback_msg.data = help_text
        self.voice_feedback_publisher.publish(feedback_msg)

    def activate_voice_control(self):
        """Activate voice control"""
        self.voice_control_active = True
        self.get_logger().info("Voice control activated")
        self.publish_status("VOICE_CONTROL_ACTIVATED")

    def deactivate_voice_control(self):
        """Deactivate voice control"""
        self.voice_control_active = False
        self.get_logger().info("Voice control deactivated")
        self.publish_status("VOICE_CONTROL_DEACTIVATED")

    def status_callback(self):
        """Publish periodic status updates"""
        if len(self.command_history) > 10:
            self.command_history = self.command_history[-10:]  # Keep last 10 commands

        status_msg = String()
        status_msg.data = f"Active: {self.voice_control_active}, Commands: {len(self.command_history)}"
        self.status_publisher.publish(status_msg)

    def publish_status(self, status: str):
        """Publish status message"""
        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)


def main(args=None):
    """
    Main function to run the voice robot controller
    """
    rclpy.init(args=args)

    controller = VoiceRobotController()

    try:
        controller.get_logger().info("Voice Robot Controller starting...")
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down Voice Robot Controller")
    finally:
        # Ensure robot stops before shutting down
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()