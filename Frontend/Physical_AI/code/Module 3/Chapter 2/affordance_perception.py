#!/usr/bin/env python3
"""
Affordance-Based Perception and Action System

This script implements affordance detection and action selection based on
environmental possibilities for humanoid robot interaction.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Point
from cv_bridge import CvBridge
import numpy as np
import math
from typing import Dict, List, Tuple
import json


class AffordanceSpace:
    """Represents the space of possible affordances in the environment"""

    def __init__(self):
        self.affordances = {
            'grasp': [],      # Grasp affordances
            'support': [],    # Support affordances
            'contain': [],    # Containment affordances
            'move': [],       # Move affordances
            'navigate': [],   # Navigation affordances
            'manipulate': []  # Manipulation affordances
        }
        self.affordance_weights = {
            'grasp': 1.0,
            'support': 1.0,
            'contain': 0.8,
            'move': 0.9,
            'navigate': 1.0,
            'manipulate': 0.9
        }

    def add_affordance(self, affordance_type: str, properties: Dict):
        """Add an affordance to the space"""
        if affordance_type in self.affordances:
            self.affordances[affordance_type].append(properties)

    def get_relevant_affordances(self, context: str = None) -> Dict[str, List]:
        """Get affordances relevant to current context"""
        if context is None:
            return self.affordances

        # Context-specific filtering
        relevant = {}
        for affordance_type, affordances in self.affordances.items():
            if context == 'grasping' and affordance_type in ['grasp', 'support']:
                relevant[affordance_type] = affordances
            elif context == 'navigation' and affordance_type in ['navigate', 'move']:
                relevant[affordance_type] = affordances
            elif context == 'manipulation' and affordance_type in ['grasp', 'manipulate', 'support']:
                relevant[affordance_type] = affordances
            else:
                relevant[affordance_type] = affordances

        return relevant

    def rank_affordances(self, affordance_type: str, position: Tuple[float, float, float] = None) -> List:
        """Rank affordances of a specific type"""
        if affordance_type not in self.affordances:
            return []

        affordances = self.affordances[affordance_type]
        ranked = []

        for affordance in affordances:
            # Calculate relevance score based on various factors
            score = self.affordance_weights[affordance_type]

            # Distance factor (if position provided)
            if position and 'position' in affordance:
                dist = math.sqrt(
                    sum((a - b)**2 for a, b in zip(position, affordance['position']))
                )
                score *= (1.0 / (1.0 + dist))  # Closer objects are more relevant

            # Confidence factor
            if 'confidence' in affordance:
                score *= affordance['confidence']

            # Task relevance factor
            if 'task_relevance' in affordance:
                score *= affordance['task_relevance']

            ranked.append((affordance, score))

        # Sort by score in descending order
        ranked.sort(key=lambda x: x[1], reverse=True)
        return [aff for aff, score in ranked]


class AffordanceDetector:
    """Detects affordances from sensor data"""

    def __init__(self):
        self.cv_bridge = CvBridge()
        self.affordance_space = AffordanceSpace()

    def detect_from_image(self, image, camera_info=None):
        """Detect affordances from visual input"""
        # This would use computer vision models in a real implementation
        # For simulation, we'll return some affordances

        # Simulate affordance detection
        detected_affordances = {
            'grasp': [
                {'position': (1.0, 0.5, 0.2), 'object': 'red_cup', 'size': (0.08, 0.08, 0.1), 'confidence': 0.85},
                {'position': (1.2, -0.3, 0.1), 'object': 'green_box', 'size': (0.1, 0.1, 0.1), 'confidence': 0.90}
            ],
            'support': [
                {'position': (0.0, 1.0, 0.0), 'surface': 'table', 'size': (1.0, 0.8, 0.8), 'confidence': 0.95},
                {'position': (-0.5, -1.0, 0.0), 'surface': 'chair', 'size': (0.5, 0.5, 0.8), 'confidence': 0.75}
            ],
            'navigate': [
                {'position': (2.0, 0.0, 0.0), 'clearance': 1.5, 'direction': 'forward', 'confidence': 0.80},
                {'position': (0.0, 2.0, 0.0), 'clearance': 1.2, 'direction': 'left', 'confidence': 0.70}
            ]
        }

        # Add detected affordances to space
        for affordance_type, affordances in detected_affordances.items():
            for affordance in affordances:
                self.affordance_space.add_affordance(affordance_type, affordance)

        return detected_affordances

    def detect_from_lidar(self, lidar_msg):
        """Detect affordances from LiDAR data"""
        affordances = []

        # Process LiDAR data to detect navigable spaces and obstacles
        if lidar_msg.ranges:
            # Detect free space for navigation
            for i, range_val in enumerate(lidar_msg.ranges):
                if 0.5 < range_val < 5.0:  # Valid range for navigation
                    angle = lidar_msg.angle_min + i * lidar_msg.angle_increment
                    x = range_val * math.cos(angle)
                    y = range_val * math.sin(angle)

                    affordance = {
                        'position': (x, y, 0.0),
                        'distance': range_val,
                        'angle': angle,
                        'type': 'navigable',
                        'confidence': min(range_val / 5.0, 1.0)  # Higher confidence for closer clearances
                    }
                    affordances.append(affordance)
                    self.affordance_space.add_affordance('navigate', affordance)

        return affordances


class AffordanceActionSelector:
    """Selects actions based on detected affordances"""

    def __init__(self):
        self.affordance_space = AffordanceSpace()

    def select_action(self, current_task: str, robot_position: Tuple[float, float, float]):
        """Select the most appropriate action based on affordances and task"""
        # Get relevant affordances for the current task
        relevant_affordances = self.affordance_space.get_relevant_affordances(current_task)

        if current_task == 'grasping':
            return self._select_grasping_action(relevant_affordances, robot_position)
        elif current_task == 'navigation':
            return self._select_navigation_action(relevant_affordances, robot_position)
        elif current_task == 'manipulation':
            return self._select_manipulation_action(relevant_affordances, robot_position)
        else:
            return self._select_exploration_action(relevant_affordances, robot_position)

    def _select_grasping_action(self, affordances, robot_position):
        """Select grasping action based on grasp affordances"""
        graspable_objects = affordances.get('grasp', [])
        if not graspable_objects:
            return {'action': 'search', 'target': 'graspable_object'}

        # Rank graspable objects by relevance
        ranked_objects = self.affordance_space.rank_affordances('grasp', robot_position)
        if ranked_objects:
            target = ranked_objects[0]  # Highest ranked object
            return {
                'action': 'grasp',
                'target': target,
                'position': target['position'],
                'object': target['object']
            }

        return {'action': 'idle', 'reason': 'no_suitable_objects'}

    def _select_navigation_action(self, affordances, robot_position):
        """Select navigation action based on navigable affordances"""
        navigable_paths = affordances.get('navigate', [])
        if not navigable_paths:
            return {'action': 'rotate', 'direction': 'search'}

        # Rank navigation options
        ranked_paths = self.affordance_space.rank_affordances('navigate', robot_position)
        if ranked_paths:
            best_path = ranked_paths[0]  # Highest ranked path
            return {
                'action': 'navigate',
                'target': best_path['position'],
                'direction': best_path.get('direction', 'forward'),
                'clearance': best_path.get('clearance', 1.0)
            }

        return {'action': 'idle', 'reason': 'no_clear_paths'}

    def _select_manipulation_action(self, affordances, robot_position):
        """Select manipulation action based on manipulation affordances"""
        # Look for graspable objects near support surfaces
        graspable = affordances.get('grasp', [])
        support = affordances.get('support', [])

        for obj in graspable:
            for surface in support:
                # Check if object is on or near a support surface
                obj_pos = obj['position']
                surf_pos = surface['position']
                distance = math.sqrt(sum((a - b)**2 for a, b in zip(obj_pos, surf_pos)))
                if distance < 0.5:  # Object is near surface
                    return {
                        'action': 'manipulate',
                        'target_object': obj,
                        'support_surface': surface,
                        'operation': 'move_to_surface'
                    }

        # If no object-surface pairs found, look for graspable objects
        if graspable:
            ranked_objects = self.affordance_space.rank_affordances('grasp', robot_position)
            if ranked_objects:
                return {
                    'action': 'grasp',
                    'target': ranked_objects[0],
                    'object': ranked_objects[0]['object']
                }

        return {'action': 'search', 'target': 'manipulable_objects'}

    def _select_exploration_action(self, affordances, robot_position):
        """Select exploration action when task is not specified"""
        # Prioritize based on affordance availability
        if affordances.get('grasp'):
            return self._select_grasping_action(affordances, robot_position)
        elif affordances.get('navigate'):
            return self._select_navigation_action(affordances, robot_position)
        elif affordances.get('support'):
            return {'action': 'approach', 'target': affordances['support'][0]['position']}
        else:
            return {'action': 'explore', 'direction': 'random'}


class AffordancePerceptionNode(Node):
    """ROS 2 node for affordance-based perception and action"""

    def __init__(self):
        super().__init__('affordance_perception')

        # Initialize affordance systems
        self.affordance_detector = AffordanceDetector()
        self.action_selector = AffordanceActionSelector()
        self.cv_bridge = CvBridge()

        # Subscribe to sensor data
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

        self.command_subscription = self.create_subscription(
            String,
            '/robot/command',
            self.command_callback,
            10
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.response_publisher = self.create_publisher(String, '/robot/response', 10)

        # Timer for action execution
        self.timer = self.create_timer(0.1, self.action_execution_loop)

        # Robot state
        self.current_task = 'exploration'
        self.robot_position = (0.0, 0.0, 0.0)
        self.last_action = None

        self.get_logger().info('Affordance Perception Node initialized')

    def command_callback(self, msg):
        """Process commands to set current task"""
        command = msg.data.lower()
        self.get_logger().info(f'Received command: {command}')

        if 'grasp' in command or 'pick' in command:
            self.current_task = 'grasping'
        elif 'navigate' in command or 'go' in command:
            self.current_task = 'navigation'
        elif 'manipulate' in command or 'move' in command:
            self.current_task = 'manipulation'
        else:
            self.current_task = 'exploration'

        response = f"Task set to {self.current_task}. Analyzing affordances in environment."
        self.publish_response(response)

    def image_callback(self, msg):
        """Process visual affordances"""
        try:
            affordances = self.affordance_detector.detect_from_image(msg)
            self.get_logger().debug(f'Detected {sum(len(v) for v in affordances.values())} affordances from image')
        except Exception as e:
            self.get_logger().error(f'Error processing image affordances: {e}')

    def lidar_callback(self, msg):
        """Process spatial affordances"""
        affordances = self.affordance_detector.detect_from_lidar(msg)
        self.get_logger().debug(f'Detected {len(affordances)} spatial affordances from LiDAR')

    def action_execution_loop(self):
        """Main loop for selecting and executing affordance-based actions"""
        # Select action based on current task and detected affordances
        action = self.action_selector.select_action(self.current_task, self.robot_position)

        if action['action'] != 'idle':
            self.execute_action(action)

    def execute_action(self, action):
        """Execute the selected action"""
        action_type = action['action']
        self.get_logger().info(f'Executing action: {action_type}')

        cmd = Twist()

        if action_type == 'grasp':
            # Move towards the object to grasp
            target_pos = action['position']
            cmd.linear.x = 0.2  # Slow approach for grasping
            cmd.angular.z = 0.0
            self.get_logger().info(f'Approaching object at {target_pos} for grasping')

        elif action_type == 'navigate':
            # Navigate towards the target
            cmd.linear.x = 0.4  # Navigation speed
            cmd.angular.z = 0.0
            self.get_logger().info(f'Navigating to {action.get("target", "unknown")}')

        elif action_type == 'manipulate':
            # Manipulation action
            cmd.linear.x = 0.1  # Careful movement for manipulation
            cmd.angular.z = 0.0
            self.get_logger().info(f'Manipulating object {action.get("object", "unknown")}')

        elif action_type == 'approach':
            # Approach action
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            self.get_logger().info(f'Approaching {action.get("target", "unknown")}')

        elif action_type == 'rotate':
            # Rotate to search
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Rotate in place
            self.get_logger().info('Rotating to search for affordances')

        elif action_type == 'explore':
            # Random exploration
            cmd.linear.x = 0.2
            cmd.angular.z = 0.3 if action.get('direction') == 'random' else 0.0
            self.get_logger().info('Exploring environment')

        # Publish the command
        self.cmd_vel_publisher.publish(cmd)
        self.last_action = action

        # Publish response
        response = f"Executing {action_type} action based on environmental affordances."
        self.publish_response(response)

    def publish_response(self, response_text):
        """Publish AI response to user"""
        response_msg = String()
        response_msg.data = response_text
        self.response_publisher.publish(response_msg)
        self.get_logger().info(f'AI Response: {response_text}')


def main(args=None):
    rclpy.init(args=args)

    node = AffordancePerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Affordance Perception Node')
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