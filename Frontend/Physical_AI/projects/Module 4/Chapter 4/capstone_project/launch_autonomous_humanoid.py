#!/usr/bin/env python3
"""
Launch Script for Autonomous Humanoid System

This script demonstrates the complete autonomous humanoid system
integrating all components from the course.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading
from typing import List, Dict, Any


class AutonomousHumanoidDemo(Node):
    """
    Demo node that demonstrates the autonomous humanoid capabilities
    """

    def __init__(self):
        super().__init__('autonomous_humanoid_demo')

        # Publisher for voice commands
        self.voice_command_publisher = self.create_publisher(
            String,
            '/voice_commands',
            10
        )

        # Publisher for system status requests
        self.status_request_publisher = self.create_publisher(
            String,
            '/autonomous_humanoid/status_request',
            10
        )

        # Subscriber for system feedback
        self.feedback_subscription = self.create_subscription(
            String,
            '/autonomous_humanoid/feedback',
            self.feedback_callback,
            10
        )

        # Demo commands to execute
        self.demo_commands = [
            "Hello, autonomous humanoid",
            "Find the red cube",
            "Move forward",
            "Turn left",
            "Stop moving",
            "Where are you?",
            "What can you do?"
        ]

        self.command_index = 0
        self.demo_active = False

        self.get_logger().info('Autonomous Humanoid Demo initialized')

    def feedback_callback(self, msg: String):
        """Handle feedback from the autonomous humanoid system"""
        self.get_logger().info(f"System feedback: {msg.data}")

    def start_demo(self):
        """Start the demonstration sequence"""
        self.demo_active = True
        self.get_logger().info("Starting Autonomous Humanoid demonstration...")

        # Send demo commands in sequence
        for command in self.demo_commands:
            if not self.demo_active:
                break

            self.get_logger().info(f"Sending command: '{command}'")

            cmd_msg = String()
            cmd_msg.data = command
            self.voice_command_publisher.publish(cmd_msg)

            # Wait between commands
            time.sleep(3.0)

        self.get_logger().info("Demonstration completed")

    def stop_demo(self):
        """Stop the demonstration"""
        self.demo_active = False
        self.get_logger().info("Demonstration stopped")


def main(args=None):
    """
    Main function to run the autonomous humanoid demonstration
    """
    rclpy.init(args=args)

    demo_node = AutonomousHumanoidDemo()

    try:
        demo_node.get_logger().info("Starting Autonomous Humanoid demonstration...")

        # Start the demo in a separate thread to allow for Ctrl+C handling
        demo_thread = threading.Thread(target=demo_node.start_demo)
        demo_thread.start()

        # Run the node to handle feedback
        while rclpy.ok() and demo_thread.is_alive():
            rclpy.spin_once(demo_node, timeout_sec=0.1)

        demo_thread.join()

    except KeyboardInterrupt:
        demo_node.get_logger().info("Shutting down Autonomous Humanoid Demo")
        demo_node.stop_demo()
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()