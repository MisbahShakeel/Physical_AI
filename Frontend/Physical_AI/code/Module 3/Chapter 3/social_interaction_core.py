#!/usr/bin/env python3
"""
Core Social Interaction System for Humanoid Robot

This script implements the core components of social interaction and social cognition
for humanoid robots, including social signal processing, theory of mind, and social decision making.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import numpy as np
import time
import math
from enum import Enum
from typing import Dict, List, Tuple, Optional
import json


class SocialState(Enum):
    IDLE = "idle"
    PERCEIVING_SOCIAL = "perceiving_social"
    INTERPRETING = "interpreting"
    DECIDING = "deciding"
    RESPONDING = "responding"
    LEARNING_SOCIAL = "learning_social"


class SocialSignalProcessor:
    """Processes social signals from various modalities"""

    def __init__(self):
        self.cv_bridge = CvBridge()
        self.social_signals = {
            'gaze': {'direction': (0, 0, 1), 'duration': 0.0, 'target': None},
            'gesture': {'type': 'none', 'intensity': 0.0, 'timestamp': 0.0},
            'proximity': {'distance': float('inf'), 'person_count': 0},
            'vocal': {'volume': 0.0, 'tone': 'neutral', 'words': []}
        }
        self.social_attention_map = np.zeros((10, 10))  # 10x10 grid of attention

    def process_visual_social_signals(self, image):
        """Process visual social signals like gaze and gestures"""
        # Simulate detection of social signals from image
        # In real implementation, this would use computer vision models
        social_info = {
            'gaze_detected': True,
            'gaze_direction': (0.5, 0.3, 1.0),  # Normalized direction vector
            'gesture_type': 'wave',  # Detected gesture
            'person_count': 1,  # Number of people detected
            'face_confidence': 0.9  # Confidence in face detection
        }

        # Update social signals
        self.social_signals['gaze']['direction'] = social_info['gaze_direction']
        self.social_signals['gaze']['target'] = 'human' if social_info['person_count'] > 0 else None
        self.social_signals['gesture']['type'] = social_info['gesture_type']
        self.social_signals['gesture']['intensity'] = social_info['face_confidence']
        self.social_signals['gesture']['timestamp'] = time.time()
        self.social_signals['proximity']['person_count'] = social_info['person_count']

        return social_info

    def process_proximity_signals(self, lidar_data):
        """Process proximity-based social signals from LiDAR"""
        if not lidar_data.ranges:
            return

        # Detect people based on proximity patterns
        person_distances = []
        for i, range_val in enumerate(lidar_data.ranges):
            if 0.5 < range_val < 3.0:  # Potential person distance
                angle = lidar_data.angle_min + i * lidar_data.angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)

                # Simple clustering to detect people
                if range_val < 1.5:  # Close enough to be a person
                    person_distances.append(range_val)

        self.social_signals['proximity']['distance'] = min(person_distances) if person_distances else float('inf')
        self.social_signals['proximity']['person_count'] = len(person_distances)

        return {
            'person_distances': person_distances,
            'closest_person': min(person_distances) if person_distances else float('inf'),
            'person_count': len(person_distances)
        }

    def get_social_attention_focus(self) -> Tuple[float, float]:
        """Get the current social attention focus"""
        # Based on gaze direction and proximity
        if self.social_signals['gaze']['target'] is not None:
            # Return gaze direction as focus point
            return (self.social_signals['gaze']['direction'][0],
                   self.social_signals['gaze']['direction'][1])
        elif self.social_signals['proximity']['person_count'] > 0:
            # Focus on closest person
            return (0.0, 0.0)  # Simplified - would use actual person position
        else:
            return (0.0, 0.0)  # Default focus


class TheoryOfMindSystem:
    """Implements theory of mind capabilities for understanding human mental states"""

    def __init__(self):
        self.human_models = {}  # Models of different humans
        self.belief_states = {}  # Human belief states
        self.desire_models = {}  # Human desire/need models
        self.intention_predictor = {}  # Models for predicting human intentions

    def update_human_model(self, human_id: str, observed_behavior: Dict):
        """Update model of a specific human based on observed behavior"""
        if human_id not in self.human_models:
            self.human_models[human_id] = {
                'preferences': {},
                'personality': 'neutral',
                'interaction_history': [],
                'knowledge_state': {}  # What they know
            }

        # Update interaction history
        self.human_models[human_id]['interaction_history'].append({
            'behavior': observed_behavior,
            'timestamp': time.time()
        })

        # Update belief state about what human knows
        self._update_belief_state(human_id, observed_behavior)

        # Update desire/need model
        self._update_desire_model(human_id, observed_behavior)

    def _update_belief_state(self, human_id: str, behavior: Dict):
        """Update beliefs about what the human believes/knows"""
        if human_id not in self.belief_states:
            self.belief_states[human_id] = {
                'knowledge': set(),  # Things human knows
                'beliefs': {},  # Human's beliefs about world
                'attention': set()  # What human is attending to
            }

        # Update based on behavior
        if 'gaze' in behavior:
            self.belief_states[human_id]['attention'].add('object_looked_at')

        if 'gesture' in behavior:
            self.belief_states[human_id]['knowledge'].add('intention_signalled')

    def _update_desire_model(self, human_id: str, behavior: Dict):
        """Update model of human's desires and needs"""
        if human_id not in self.desire_models:
            self.desire_models[human_id] = {
                'needs': [],  # Current needs
                'preferences': {},  # Preferences
                'goals': []  # Current goals
            }

        # Infer needs from behavior
        if behavior.get('gesture_type') == 'pointing':
            self.desire_models[human_id]['needs'].append('assistance')
        elif behavior.get('gaze_duration', 0) > 2.0:
            self.desire_models[human_id]['needs'].append('information')

    def predict_intention(self, human_id: str, current_context: Dict) -> str:
        """Predict human intention based on behavior and context"""
        if human_id not in self.human_models:
            return 'unknown'

        # Simple intention prediction based on recent behavior
        recent_behaviors = self.human_models[human_id]['interaction_history'][-5:]
        if not recent_behaviors:
            return 'idle'

        # Look for patterns in recent behavior
        gesture_types = [b['behavior'].get('gesture_type', 'none') for b in recent_behaviors]
        gaze_directions = [b['behavior'].get('gaze_direction', (0, 0, 1)) for b in recent_behaviors]

        # Simple pattern matching
        if 'wave' in gesture_types:
            return 'greeting'
        elif 'pointing' in gesture_types:
            return 'requesting_assistance'
        elif any('approach' in str(gd) for gd in gaze_directions):
            return 'requesting_attention'
        else:
            return 'unknown'

    def get_human_perspective(self, human_id: str, topic: str) -> Dict:
        """Get the human's perspective on a topic"""
        if human_id not in self.belief_states:
            return {'knowledge': False, 'attitude': 'neutral', 'understanding': 'low'}

        belief_state = self.belief_states[human_id]
        knows_topic = topic in belief_state['knowledge']

        return {
            'knowledge': knows_topic,
            'attitude': 'positive' if knows_topic else 'neutral',
            'understanding': 'high' if knows_topic else 'low'
        }


class SocialDecisionMaker:
    """Makes social decisions considering human preferences and social norms"""

    def __init__(self):
        self.social_norms = {
            'personal_space': 1.0,  # meters
            'greeting_protocol': True,
            'eye_contact_duration': 2.0,  # seconds
            'response_time': 3.0  # seconds
        }
        self.social_preferences = {}  # Human-specific preferences
        self.cultural_contexts = {}  # Cultural adaptation parameters

    def decide_social_action(self, human_id: str, context: Dict, theory_of_mind: TheoryOfMindSystem) -> Dict:
        """Decide on appropriate social action based on context and human model"""
        # Get human information
        human_perspective = theory_of_mind.get_human_perspective(human_id, 'current_interaction')
        predicted_intention = theory_of_mind.predict_intention(human_id, context)

        # Consider social norms
        current_distance = context.get('distance_to_human', float('inf'))
        needs_to_maintain_distance = current_distance < self.social_norms['personal_space']

        # Consider human preferences and needs
        human_needs = self._get_human_needs(human_id, theory_of_mind)
        human_preferences = self._get_human_preferences(human_id)

        # Decide action based on intention, needs, and norms
        action = self._select_action_based_on_context(
            predicted_intention, human_needs, needs_to_maintain_distance, human_preferences
        )

        return action

    def _get_human_needs(self, human_id: str, tom_system: TheoryOfMindSystem) -> List[str]:
        """Get current human needs based on theory of mind system"""
        if human_id in tom_system.desire_models:
            return tom_system.desire_models[human_id].get('needs', [])
        return []

    def _get_human_preferences(self, human_id: str) -> Dict:
        """Get human preferences"""
        if human_id in self.social_preferences:
            return self.social_preferences[human_id]
        return {'communication_style': 'formal', 'pace': 'moderate'}

    def _select_action_based_on_context(self, intention: str, needs: List[str],
                                      maintain_distance: bool, preferences: Dict) -> Dict:
        """Select appropriate action based on context"""
        action = {'type': 'none', 'parameters': {}}

        if intention == 'greeting':
            action = {
                'type': 'greet',
                'parameters': {
                    'style': preferences.get('communication_style', 'friendly'),
                    'distance_maintained': maintain_distance
                }
            }
        elif intention == 'requesting_assistance':
            action = {
                'type': 'assist',
                'parameters': {
                    'approach': not maintain_distance,
                    'ask_clarifying_questions': True
                }
            }
        elif intention == 'requesting_attention':
            action = {
                'type': 'acknowledge',
                'parameters': {
                    'gaze_direction': 'toward_human',
                    'verbal_acknowledgment': True
                }
            }
        else:
            # Default social behavior
            action = {
                'type': 'maintain_presence',
                'parameters': {
                    'polite_attention': True,
                    'respect_personal_space': maintain_distance
                }
            }

        return action


class SocialInteractionNode(Node):
    """Main social interaction node for humanoid robot"""

    def __init__(self):
        super().__init__('social_interaction_node')

        # Initialize social systems
        self.social_processor = SocialSignalProcessor()
        self.theory_of_mind = TheoryOfMindSystem()
        self.social_decision_maker = SocialDecisionMaker()
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

        self.social_command_subscription = self.create_subscription(
            String,
            '/social/command',
            self.social_command_callback,
            10
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.response_publisher = self.create_publisher(String, '/robot/response', 10)
        self.social_state_publisher = self.create_publisher(String, '/social/state', 10)

        # Timer for social processing loop
        self.social_timer = self.create_timer(0.1, self.social_processing_loop)

        # Robot state
        self.current_social_state = SocialState.IDLE
        self.detected_humans = {}  # Track detected humans
        self.last_interaction_time = time.time()
        self.social_context = {}
        self.active_human_id = None

        self.get_logger().info('Social Interaction Node initialized')

    def command_callback(self, msg):
        """Process general commands"""
        command = msg.data.lower()
        self.get_logger().info(f'Received general command: {command}')

        if 'social' in command:
            self.current_social_state = SocialState.PERCEIVING_SOCIAL
            response = "Entering social interaction mode. Detecting and responding to humans."
            self.publish_response(response)

    def social_command_callback(self, msg):
        """Process social-specific commands"""
        command = msg.data.lower()
        self.get_logger().info(f'Received social command: {command}')

        if 'greet' in command or 'hello' in command:
            self._execute_greeting()
        elif 'follow' in command:
            self._execute_follow_behavior()
        elif 'assist' in command:
            self._execute_assistance_behavior()

    def image_callback(self, msg):
        """Process visual social signals"""
        try:
            social_info = self.social_processor.process_visual_social_signals(msg)
            self.social_context['visual'] = social_info

            # Update human tracking
            if social_info['person_count'] > 0:
                self._track_humans(social_info)

            self.get_logger().debug(f'Detected {social_info["person_count"]} people in visual field')

        except Exception as e:
            self.get_logger().error(f'Error processing visual social signals: {e}')

    def lidar_callback(self, msg):
        """Process proximity-based social signals"""
        proximity_info = self.social_processor.process_proximity_signals(msg)
        self.social_context['proximity'] = proximity_info

        # Update human tracking based on proximity
        if proximity_info['person_count'] > 0:
            self._update_human_proximity(proximity_info)

    def _track_humans(self, visual_info):
        """Track humans detected visually"""
        current_time = time.time()

        for i in range(visual_info['person_count']):
            human_id = f"human_{i}"
            if human_id not in self.detected_humans:
                self.detected_humans[human_id] = {
                    'first_seen': current_time,
                    'last_seen': current_time,
                    'interaction_count': 0
                }
            else:
                self.detected_humans[human_id]['last_seen'] = current_time

        # Set active human if none is set
        if not self.active_human_id and visual_info['person_count'] > 0:
            self.active_human_id = "human_0"

    def _update_human_proximity(self, proximity_info):
        """Update human tracking based on proximity"""
        # This would update position information in a real implementation
        pass

    def social_processing_loop(self):
        """Main social processing loop"""
        current_time = time.time()

        if self.active_human_id:
            # Update human model with current context
            self.theory_of_mind.update_human_model(
                self.active_human_id,
                self.social_context
            )

            # Make social decision
            if self.social_context:
                social_action = self.social_decision_maker.decide_social_action(
                    self.active_human_id,
                    self.social_context,
                    self.theory_of_mind
                )

                # Execute social action
                self._execute_social_action(social_action)

        # Publish social state
        social_state_msg = String()
        social_state_msg.data = json.dumps({
            'active_human': self.active_human_id,
            'social_state': self.current_social_state.value,
            'detected_humans': len(self.detected_humans),
            'last_interaction': self.last_interaction_time
        })
        self.social_state_publisher.publish(social_state_msg)

    def _execute_social_action(self, action: Dict):
        """Execute the decided social action"""
        action_type = action['type']
        parameters = action['parameters']

        self.get_logger().info(f'Executing social action: {action_type}')

        cmd = Twist()

        if action_type == 'greet':
            # Move appropriately based on personal space preference
            if not parameters.get('distance_maintained', False):
                cmd.linear.x = 0.2  # Approach gently
            self._publish_greeting_response(parameters)

        elif action_type == 'assist':
            # Approach to assist
            if parameters.get('approach', True):
                cmd.linear.x = 0.1  # Slow approach
            self._publish_assistance_response(parameters)

        elif action_type == 'acknowledge':
            # Acknowledge with gaze and movement
            cmd.angular.z = 0.1  # Slight turn toward human
            self._publish_acknowledgment_response(parameters)

        elif action_type == 'maintain_presence':
            # Maintain polite distance
            cmd.linear.x = 0.0
            self._publish_presence_response(parameters)

        # Publish movement command
        self.cmd_vel_publisher.publish(cmd)

    def _publish_greeting_response(self, parameters):
        """Publish greeting response"""
        greeting_style = parameters.get('style', 'friendly')
        if greeting_style == 'friendly':
            response = "Hello! It's nice to meet you."
        elif greeting_style == 'formal':
            response = "Good day. How may I assist you?"
        else:
            response = "Hello. I'm here to help."

        self.publish_response(response)

    def _publish_assistance_response(self, parameters):
        """Publish assistance response"""
        if parameters.get('ask_clarifying_questions', False):
            response = "I can help with that. Could you tell me more about what you need?"
        else:
            response = "I'm here to assist you. What would you like help with?"

        self.publish_response(response)

    def _publish_acknowledgment_response(self, parameters):
        """Publish acknowledgment response"""
        response = "I see you and acknowledge your presence."
        self.publish_response(response)

    def _publish_presence_response(self, parameters):
        """Publish presence response"""
        response = "I'm here and paying attention to the interaction."
        self.publish_response(response)

    def _execute_greeting(self):
        """Execute explicit greeting behavior"""
        if self.active_human_id:
            self.get_logger().info(f'Greeting human {self.active_human_id}')
            response = f"Hello! I see you there. It's nice to meet you!"
            self.publish_response(response)

            # Gentle approach while maintaining personal space
            cmd = Twist()
            cmd.linear.x = 0.1
            self.cmd_vel_publisher.publish(cmd)

    def _execute_follow_behavior(self):
        """Execute follow behavior"""
        if self.active_human_id:
            self.get_logger().info(f'Following human {self.active_human_id}')
            response = "I'll follow you now. Please let me know if I get too close or too far."
            self.publish_response(response)

    def _execute_assistance_behavior(self):
        """Execute assistance behavior"""
        if self.active_human_id:
            self.get_logger().info(f'Assisting human {self.active_human_id}')
            response = "I'm ready to assist you. What do you need help with?"
            self.publish_response(response)

    def publish_response(self, response_text):
        """Publish AI response to user"""
        response_msg = String()
        response_msg.data = response_text
        self.response_publisher.publish(response_msg)
        self.get_logger().info(f'Social Response: {response_text}')


def main(args=None):
    rclpy.init(args=args)

    node = SocialInteractionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Social Interaction Node')
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