#!/usr/bin/env python3
"""
Trust Building and Social Interaction Demo

This project demonstrates trust building mechanisms and social interaction
capabilities for humanoid robots, including social learning, adaptation,
and long-term relationship building.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int8
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PointStamped
from cv_bridge import CvBridge
import numpy as np
import time
import random
from enum import Enum
from typing import Dict, List, Tuple, Optional
import json
import pickle
import os


class TrustLevel(Enum):
    UNKNOWN = 0
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    VERY_HIGH = 4


class InteractionType(Enum):
    GREETING = "greeting"
    ASSISTANCE = "assistance"
    CONVERSATION = "conversation"
    FOLLOW = "follow"
    LEARN = "learn"
    ERROR = "error"


class TrustModel:
    """Models and tracks trust relationships with humans"""

    def __init__(self):
        self.human_trust_models = {}  # Maps human_id to trust model
        self.trust_decay_rate = 0.99  # Trust decays over time
        self.positive_reinforcement = 0.1
        self.negative_penalty = -0.2
        self.error_penalty = -0.3

    def get_trust_level(self, human_id: str) -> TrustLevel:
        """Get current trust level for a human"""
        if human_id not in self.human_trust_models:
            return TrustLevel.UNKNOWN

        trust_score = self.human_trust_models[human_id]['trust_score']
        if trust_score < 0.2:
            return TrustLevel.LOW
        elif trust_score < 0.5:
            return TrustLevel.MEDIUM
        elif trust_score < 0.8:
            return TrustLevel.HIGH
        else:
            return TrustLevel.VERY_HIGH

    def update_trust(self, human_id: str, interaction_type: InteractionType, success: bool, quality: float = 1.0):
        """Update trust based on interaction outcome"""
        if human_id not in self.human_trust_models:
            self.human_trust_models[human_id] = {
                'trust_score': 0.5,  # Start with neutral trust
                'interactions': [],
                'last_interaction': time.time(),
                'error_count': 0,
                'success_count': 0
            }

        model = self.human_trust_models[human_id]

        # Apply decay since last interaction
        time_since = time.time() - model['last_interaction']
        decay_factor = self.trust_decay_rate ** (time_since / 3600)  # Decay per hour
        model['trust_score'] *= decay_factor

        # Update based on interaction
        if success:
            model['trust_score'] += self.positive_reinforcement * quality
            model['success_count'] += 1
        else:
            if interaction_type == InteractionType.ERROR:
                model['error_count'] += 1
                model['trust_score'] += self.error_penalty
            else:
                model['trust_score'] += self.negative_penalty

        # Ensure trust stays within bounds
        model['trust_score'] = max(0.0, min(1.0, model['trust_score']))
        model['last_interaction'] = time.time()

        # Log interaction
        model['interactions'].append({
            'type': interaction_type.value,
            'success': success,
            'quality': quality,
            'timestamp': time.time(),
            'trust_after': model['trust_score']
        })

        # Keep interaction history manageable
        if len(model['interactions']) > 100:
            model['interactions'] = model['interactions'][-50:]

    def get_interaction_history(self, human_id: str, limit: int = 10) -> List[Dict]:
        """Get recent interaction history for a human"""
        if human_id in self.human_trust_models:
            interactions = self.human_trust_models[human_id]['interactions']
            return interactions[-limit:] if len(interactions) > limit else interactions
        return []

    def get_trust_trend(self, human_id: str) -> str:
        """Get trust trend (increasing, decreasing, stable)"""
        history = self.get_interaction_history(human_id, 5)
        if len(history) < 2:
            return "unknown"

        recent_trust = [h['trust_after'] for h in history]
        if len(recent_trust) >= 2:
            if recent_trust[-1] > recent_trust[0]:
                return "increasing"
            elif recent_trust[-1] < recent_trust[0]:
                return "decreasing"
            else:
                return "stable"

        return "unknown"


class SocialAdaptationSystem:
    """Adapts robot behavior based on social feedback and trust levels"""

    def __init__(self):
        self.human_preferences = {}  # Maps human_id to preferences
        self.social_strategies = {
            TrustLevel.LOW: {
                'distance': 2.0,  # Keep more distance
                'interaction_style': 'formal',
                'communication_speed': 'slow',
                'initiative_level': 'low'
            },
            TrustLevel.MEDIUM: {
                'distance': 1.5,
                'interaction_style': 'polite',
                'communication_speed': 'moderate',
                'initiative_level': 'medium'
            },
            TrustLevel.HIGH: {
                'distance': 1.0,
                'interaction_style': 'friendly',
                'communication_speed': 'moderate',
                'initiative_level': 'medium'
            },
            TrustLevel.VERY_HIGH: {
                'distance': 0.8,
                'interaction_style': 'warm',
                'communication_speed': 'normal',
                'initiative_level': 'high'
            }
        }

    def adapt_behavior(self, human_id: str, trust_level: TrustLevel) -> Dict:
        """Adapt behavior based on trust level and preferences"""
        if human_id not in self.human_preferences:
            self.human_preferences[human_id] = {
                'preferred_distance': 1.0,
                'communication_style': 'neutral',
                'interaction_frequency': 'moderate',
                'personality_match': 'unknown'
            }

        # Get base strategy from trust level
        base_strategy = self.social_strategies.get(trust_level, self.social_strategies[TrustLevel.MEDIUM])

        # Apply individual preferences
        user_prefs = self.human_preferences[human_id]
        adapted_behavior = base_strategy.copy()

        # Adjust based on user preferences
        if user_prefs['preferred_distance'] > 0:
            adapted_behavior['distance'] = user_prefs['preferred_distance']

        if user_prefs['communication_style'] != 'neutral':
            adapted_behavior['interaction_style'] = user_prefs['communication_style']

        return adapted_behavior

    def learn_preference(self, human_id: str, feedback: Dict):
        """Learn human preferences from feedback"""
        if human_id not in self.human_preferences:
            self.human_preferences[human_id] = {
                'preferred_distance': 1.0,
                'communication_style': 'neutral',
                'interaction_frequency': 'moderate',
                'personality_match': 'unknown'
            }

        prefs = self.human_preferences[human_id]

        # Update preferences based on feedback
        if 'distance_comfort' in feedback:
            # Adjust preferred distance based on comfort feedback
            if feedback['distance_comfort'] == 'too_close':
                prefs['preferred_distance'] = min(2.0, prefs['preferred_distance'] + 0.2)
            elif feedback['distance_comfort'] == 'too_far':
                prefs['preferred_distance'] = max(0.5, prefs['preferred_distance'] - 0.2)

        if 'communication_style' in feedback:
            prefs['communication_style'] = feedback['communication_style']

        if 'response_speed' in feedback:
            # Could map to communication speed preference
            pass


class TrustBuildingDemo(Node):
    """Complete trust building and social interaction demonstration"""

    def __init__(self):
        super().__init__('trust_building_demo')

        # Initialize trust and adaptation systems
        self.trust_model = TrustModel()
        self.adaptation_system = SocialAdaptationSystem()
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

        self.social_feedback_subscription = self.create_subscription(
            String,
            '/social/feedback',
            self.social_feedback_callback,
            10
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.response_publisher = self.create_publisher(String, '/robot/response', 10)
        self.trust_publisher = self.create_publisher(String, '/social/trust', 10)

        # Timer for social loop
        self.social_timer = self.create_timer(0.1, self.social_loop)
        self.behavior_timer = self.create_timer(1.0, self.behavior_assessment)

        # Robot state
        self.detected_humans = {}  # Track detected humans
        self.active_human_id = None
        self.interaction_history = []
        self.robot_position = (0.0, 0.0, 0.0)
        self.human_positions = []
        self.last_behavior_update = time.time()
        self.simulation_mode = True  # For demo purposes

        self.get_logger().info('Trust Building Demo initialized')

    def command_callback(self, msg):
        """Process commands for trust building"""
        command = msg.data.lower()
        self.get_logger().info(f'Received command: {command}')

        if 'trust' in command or 'build' in command:
            if self.active_human_id:
                # Update trust positively
                self.trust_model.update_trust(
                    self.active_human_id,
                    InteractionType.GREETING,
                    success=True,
                    quality=0.8
                )
                response = f"Working on building trust with {self.active_human_id}. Current trust level: {self.trust_model.get_trust_level(self.active_human_id).name.lower()}."
            else:
                response = "I don't see anyone to build trust with. Please come into view."
        elif 'follow' in command:
            if self.active_human_id:
                response = f"I can follow you, {self.active_human_id}. My trust level with you is {self.trust_model.get_trust_level(self.active_human_id).name.lower()}."
            else:
                response = "I don't see anyone to follow."
        elif 'help' in command:
            if self.active_human_id:
                response = f"I'm ready to help, {self.active_human_id}. Trust level: {self.trust_model.get_trust_level(self.active_human_id).name.lower()}."
            else:
                response = "I'm ready to help. Please let me know what you need."
        else:
            response = f"Command received: {command}. Current trust status unknown."

        self.publish_response(response)

    def social_feedback_callback(self, msg):
        """Process social feedback from humans"""
        try:
            feedback_data = json.loads(msg.data)
            human_id = feedback_data.get('human_id', 'unknown')
            feedback_type = feedback_data.get('type', 'general')
            content = feedback_data.get('content', '')

            self.get_logger().info(f'Received feedback from {human_id}: {feedback_type} - {content}')

            # Learn from feedback
            if feedback_type == 'preference':
                self.adaptation_system.learn_preference(human_id, feedback_data.get('details', {}))
            elif feedback_type == 'comfort':
                self.adaptation_system.learn_preference(human_id, {'distance_comfort': content})

            # Update trust based on positive/negative feedback
            if content in ['good', 'thank you', 'yes', 'pleased']:
                self.trust_model.update_trust(human_id, InteractionType.CONVERSATION, success=True, quality=0.7)
            elif content in ['bad', 'no', 'displeased', 'uncomfortable']:
                self.trust_model.update_trust(human_id, InteractionType.CONVERSATION, success=False, quality=0.3)

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in feedback: {msg.data}')

    def image_callback(self, msg):
        """Process visual data for human detection"""
        # Simulate human detection
        # In real implementation, this would use face detection/computer vision
        if self.simulation_mode:
            # Simulate detecting a human every few seconds
            if random.random() > 0.7:  # 30% chance of detecting human
                if 'human_1' not in self.detected_humans:
                    self.detected_humans['human_1'] = {
                        'first_seen': time.time(),
                        'last_seen': time.time(),
                        'trust_init': self.trust_model.get_trust_level('human_1')
                    }

                if not self.active_human_id:
                    self.active_human_id = 'human_1'

                # Update last seen
                self.detected_humans['human_1']['last_seen'] = time.time()

    def lidar_callback(self, msg):
        """Process LiDAR data for proximity"""
        human_positions = []
        for i, range_val in enumerate(msg.ranges):
            if 0.5 < range_val < 3.0:
                angle = msg.angle_min + i * msg.angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                human_positions.append((x, y, 0.0))

        self.human_positions = human_positions

    def social_loop(self):
        """Main social interaction loop"""
        if self.active_human_id:
            # Update trust model with passage of time
            current_time = time.time()

            # Simulate positive interactions periodically
            if current_time - self.last_behavior_update > 10:  # Every 10 seconds
                # Simulate successful interaction
                self.trust_model.update_trust(
                    self.active_human_id,
                    InteractionType.CONVERSATION,
                    success=True,
                    quality=random.uniform(0.6, 0.9)
                )
                self.last_behavior_update = current_time

            # Get current trust level and adapt behavior
            trust_level = self.trust_model.get_trust_level(self.active_human_id)
            adapted_behavior = self.adaptation_system.adapt_behavior(self.active_human_id, trust_level)

            # Log trust information
            self.get_logger().debug(
                f'Trust with {self.active_human_id}: {trust_level.name}, '
                f'Behavior: {adapted_behavior["interaction_style"]}, '
                f'Distance: {adapted_behavior["distance"]}'
            )

    def behavior_assessment(self):
        """Periodically assess and report behavior"""
        if self.active_human_id:
            trust_level = self.trust_model.get_trust_level(self.active_human_id)
            trust_trend = self.trust_model.get_trust_trend(self.active_human_id)

            # Publish trust information
            trust_msg = String()
            trust_msg.data = json.dumps({
                'human_id': self.active_human_id,
                'trust_level': trust_level.name.lower(),
                'trust_score': self.trust_model.human_trust_models.get(self.active_human_id, {}).get('trust_score', 0.0),
                'trust_trend': trust_trend,
                'interaction_count': len(self.trust_model.get_interaction_history(self.active_human_id))
            })
            self.trust_publisher.publish(trust_msg)

            # Generate adaptive response based on trust level
            if trust_trend == 'increasing':
                adaptive_response = f"I'm glad our interaction is going well, {self.active_human_id}! My trust in our relationship is growing."
            elif trust_trend == 'decreasing':
                adaptive_response = f"I notice we might need to work on our interaction, {self.active_human_id}. I'm here to improve our relationship."
            elif trust_level == TrustLevel.HIGH or trust_level == TrustLevel.VERY_HIGH:
                adaptive_response = f"I feel we have a good connection, {self.active_human_id}. How can I best assist you today?"
            elif trust_level == TrustLevel.LOW:
                adaptive_response = f"I'm still learning about you, {self.active_human_id}. Please let me know if I can help with anything."
            else:
                adaptive_response = f"Hello {self.active_human_id}. I'm happy to interact with you."

            self.publish_response(adaptive_response)

    def publish_response(self, response_text):
        """Publish AI response to user"""
        response_msg = String()
        response_msg.data = response_text
        self.response_publisher.publish(response_msg)
        self.get_logger().info(f'Trust-Building Response: {response_text}')


def main(args=None):
    rclpy.init(args=args)

    demo = TrustBuildingDemo()

    try:
        rclpy.spin(demo)
    except KeyboardInterrupt:
        demo.get_logger().info('Shutting down Trust Building Demo')
        # Print final trust statistics
        for human_id, model in demo.trust_model.human_trust_models.items():
            trust_level = demo.trust_model.get_trust_level(human_id)
            demo.get_logger().info(
                f'Final trust with {human_id}: {trust_level.name} '
                f'(score: {model["trust_score"]:.2f}, '
                f'successes: {model["success_count"]}, '
                f'errors: {model["error_count"]})'
            )
    finally:
        # Stop the robot before shutting down
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        demo.cmd_vel_publisher.publish(cmd)
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()