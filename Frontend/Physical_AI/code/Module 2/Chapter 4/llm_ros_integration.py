#!/usr/bin/env python3
"""
LLM-ROS 2 Integration Example

This script demonstrates how to integrate a Large Language Model (LLM) with ROS 2
for natural language command processing and task planning. This example uses a
simulated LLM interface since actual LLM integration would require API keys
and external services.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
import re


class LLMROSIntegration(Node):
    def __init__(self):
        super().__init__('llm_ros_integration')

        # Create CvBridge for image processing
        self.bridge = CvBridge()

        # Subscribe to natural language commands
        self.command_subscription = self.create_subscription(
            String,
            '/robot/command',
            self.command_callback,
            10
        )

        # Subscribe to camera feed for vision-language integration
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        # Publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for robot responses
        self.response_publisher = self.create_publisher(String, '/robot/response', 10)

        # Store latest image for vision-language processing
        self.latest_image = None

        # Simple command mapping for demonstration
        self.command_mapping = {
            'forward': self.move_forward,
            'backward': self.move_backward,
            'left': self.turn_left,
            'right': self.turn_right,
            'stop': self.stop_robot,
            'look': self.process_vision_command,
            'find': self.find_object_command
        }

        self.get_logger().info('LLM-ROS Integration node started')

    def command_callback(self, msg):
        """Process natural language command"""
        command_text = msg.data.lower()
        self.get_logger().info(f'Received command: {command_text}')

        # Simulate LLM processing - parse the command and extract intent
        intent = self.parse_command(command_text)

        if intent in self.command_mapping:
            # Execute the corresponding function
            response = self.command_mapping[intent]()
            if response:
                response_msg = String()
                response_msg.data = response
                self.response_publisher.publish(response_msg)
        else:
            # Unknown command - simulate LLM asking for clarification
            response = f"I don't understand the command: {command_text}. Can you rephrase?"
            response_msg = String()
            response_msg.data = response
            self.response_publisher.publish(response_msg)

    def image_callback(self, msg):
        """Process incoming image for vision-language integration"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().debug('Received image for vision-language processing')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def parse_command(self, command_text):
        """
        Simulate LLM command parsing - extract intent from natural language
        In a real implementation, this would use an actual LLM API
        """
        # Simple keyword-based parsing for demonstration
        if 'forward' in command_text or 'go' in command_text or 'move' in command_text:
            if any(word in command_text for word in ['back', 'backward', 'reverse']):
                return 'backward'
            else:
                return 'forward'
        elif 'backward' in command_text or 'back' in command_text:
            return 'backward'
        elif 'left' in command_text:
            return 'left'
        elif 'right' in command_text:
            return 'right'
        elif 'stop' in command_text or 'halt' in command_text:
            return 'stop'
        elif 'look' in command_text or 'see' in command_text or 'what' in command_text:
            return 'look'
        elif 'find' in command_text or 'search' in command_text:
            return 'find'
        else:
            # Default to forward if no clear intent
            return 'forward'

    def move_forward(self):
        """Move robot forward"""
        cmd = Twist()
        cmd.linear.x = 0.5  # m/s
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Moving forward')
        return "I'm moving forward."

    def move_backward(self):
        """Move robot backward"""
        cmd = Twist()
        cmd.linear.x = -0.5  # m/s
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Moving backward')
        return "I'm moving backward."

    def turn_left(self):
        """Turn robot left"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # rad/s
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Turning left')
        return "I'm turning left."

    def turn_right(self):
        """Turn robot right"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -0.5  # rad/s
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Turning right')
        return "I'm turning right."

    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Stopping robot')
        return "I've stopped."

    def process_vision_command(self):
        """Process a command that requires visual perception"""
        if self.latest_image is not None:
            # Simulate vision processing
            # In a real implementation, this would use computer vision algorithms
            # or a vision-language model to analyze the image
            response = "I can see objects in front of me. The environment looks clear."
        else:
            response = "I don't have current visual information. Please wait for camera data."

        self.get_logger().info(f'Vision processing result: {response}')
        return response

    def find_object_command(self):
        """Process a command to find a specific object"""
        # This would integrate with computer vision to find objects
        # For demonstration, we'll return a simulated response
        response = "I'm searching for objects. I found a red box approximately 2 meters ahead."
        self.get_logger().info(f'Object search result: {response}')
        return response


def main(args=None):
    rclpy.init(args=args)

    node = LLMROSIntegration()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LLM-ROS Integration node')
    finally:
        # Stop the robot before shutting down
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        node.cmd_vel_publisher.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()