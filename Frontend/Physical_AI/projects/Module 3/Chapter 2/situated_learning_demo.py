#!/usr/bin/env python3
"""
Situated Learning Demo for Embodied AI

This project demonstrates situated learning in embodied AI systems,
where humanoid robots learn through environmental interaction and
context-dependent experience.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan, JointState, Imu
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import time
import random
from enum import Enum
from typing import Dict, List, Tuple, Optional
import math
import pickle
import os


class LearningState(Enum):
    EXPLORING = "exploring"
    INTERACTING = "interacting"
    LEARNING = "learning"
    ADAPTING = "adapting"
    EXPLOITING = "exploiting"


class ExperienceBuffer:
    """Buffer for storing situated experiences"""

    def __init__(self, max_size=10000):
        self.experiences = []
        self.max_size = max_size
        self.context_tags = {}  # Maps context to experiences

    def add_experience(self, state, action, outcome, reward, context="general"):
        """Add an experience to the buffer"""
        experience = {
            'state': state,
            'action': action,
            'outcome': outcome,
            'reward': reward,
            'context': context,
            'timestamp': time.time()
        }
        self.experiences.append(experience)

        # Tag by context
        if context not in self.context_tags:
            self.context_tags[context] = []
        self.context_tags[context].append(len(self.experiences) - 1)

        # Maintain max size
        if len(self.experiences) > self.max_size:
            self.experiences.pop(0)
            # Update context tags after removal
            self._update_context_tags()

    def get_experiences_by_context(self, context: str, limit: int = None) -> List:
        """Get experiences for a specific context"""
        if context in self.context_tags:
            indices = self.context_tags[context]
            if limit:
                indices = indices[-limit:]  # Get most recent
            return [self.experiences[i] for i in indices]
        return []

    def get_similar_experiences(self, current_state, context="general", threshold=0.7) -> List:
        """Get experiences similar to current state"""
        experiences = self.get_experiences_by_context(context)
        similar = []

        for exp in experiences:
            similarity = self._calculate_state_similarity(current_state, exp['state'])
            if similarity > threshold:
                similar.append((exp, similarity))

        # Sort by similarity
        similar.sort(key=lambda x: x[1], reverse=True)
        return [exp for exp, sim in similar]

    def _calculate_state_similarity(self, state1, state2) -> float:
        """Calculate similarity between two states"""
        # Simplified similarity calculation
        # In a real system, this would use more sophisticated metrics
        similarity = 0.0
        count = 0

        if 'position' in state1 and 'position' in state2:
            pos1, pos2 = state1['position'], state2['position']
            dist = math.sqrt(sum((a - b)**2 for a, b in zip(pos1, pos2)))
            similarity += max(0, 1.0 - dist)  # Higher similarity for closer positions
            count += 1

        if 'affordances' in state1 and 'affordances' in state2:
            aff1_count = sum(len(v) for v in state1['affordances'].values())
            aff2_count = sum(len(v) for v in state2['affordances'].values())
            if aff1_count > 0 or aff2_count > 0:
                aff_similarity = 1.0 - abs(aff1_count - aff2_count) / max(aff1_count, aff2_count, 1)
                similarity += aff_similarity
                count += 1

        return similarity / count if count > 0 else 0.0

    def _update_context_tags(self):
        """Update context tags after experience removal"""
        self.context_tags = {}
        for i, exp in enumerate(self.experiences):
            context = exp['context']
            if context not in self.context_tags:
                self.context_tags[context] = []
            self.context_tags[context].append(i)


class ContextDetector:
    """Detects environmental contexts for situated learning"""

    def __init__(self):
        self.contexts = {
            'kitchen': {'objects': ['table', 'chair', 'cup', 'plate'], 'locations': []},
            'living_room': {'objects': ['sofa', 'tv', 'table'], 'locations': []},
            'bedroom': {'objects': ['bed', 'dresser', 'nightstand'], 'locations': []},
            'corridor': {'objects': ['door', 'wall', 'floor'], 'locations': []}
        }
        self.current_context = 'unknown'

    def detect_context(self, perception_data: Dict) -> str:
        """Detect current environmental context"""
        if 'objects' not in perception_data:
            return self.current_context

        object_names = [obj.get('name', '').lower() for obj in perception_data.get('objects', [])]
        object_counts = {obj: object_names.count(obj) for obj in set(object_names)}

        best_match = 'unknown'
        best_score = 0

        for context_name, context_info in self.contexts.items():
            score = 0
            for obj in context_info['objects']:
                if obj in object_names:
                    score += 1
            if score > best_score:
                best_score = score
                best_match = context_name

        self.current_context = best_match if best_score > 0 else 'unknown'
        return self.current_context


class SituatedLearner:
    """Implements situated learning for embodied agents"""

    def __init__(self):
        self.experience_buffer = ExperienceBuffer()
        self.context_detector = ContextDetector()
        self.action_preferences = {}  # Maps (context, state_feature) to action preferences
        self.learning_rate = 0.1
        self.exploration_rate = 0.3
        self.transfer_threshold = 0.8  # Threshold for knowledge transfer

    def learn_from_interaction(self, state, action, outcome, reward):
        """Learn from environmental interaction"""
        context = self.context_detector.detect_context(state.get('perception', {}))

        # Add to experience buffer
        self.experience_buffer.add_experience(state, action, outcome, reward, context)

        # Update action preferences based on outcome
        state_feature = self._extract_state_feature(state)
        action_key = f"{context}_{state_feature}_{str(action)}"

        if action_key not in self.action_preferences:
            self.action_preferences[action_key] = {'value': 0.0, 'visits': 0}

        current_pref = self.action_preferences[action_key]
        current_pref['visits'] += 1
        current_pref['value'] += self.learning_rate * (reward - current_pref['value'])

    def select_action(self, state, available_actions, exploration_bonus=True) -> Tuple:
        """Select action based on situated learning"""
        context = self.context_detector.detect_context(state.get('perception', {}))
        state_feature = self._extract_state_feature(state)

        # Apply exploration vs exploitation
        if exploration_bonus and random.random() < self.exploration_rate:
            # Explore: try new actions or actions with low visit counts
            return self._select_exploratory_action(state, available_actions, context, state_feature)
        else:
            # Exploit: use learned preferences
            return self._select_exploitative_action(state, available_actions, context, state_feature)

    def _select_exploratory_action(self, state, available_actions, context, state_feature):
        """Select action for exploration"""
        # Prioritize actions with fewer visits or unknown actions
        action_scores = []
        for action in available_actions:
            action_key = f"{context}_{state_feature}_{str(action)}"
            visits = self.action_preferences.get(action_key, {}).get('visits', 0)
            # Higher score for less visited actions (exploration bonus)
            score = 1.0 / (visits + 1)
            action_scores.append((action, score))

        # Select action with highest exploration score
        action_scores.sort(key=lambda x: x[1], reverse=True)
        return action_scores[0][0]

    def _select_exploitative_action(self, state, available_actions, context, state_feature):
        """Select action based on learned preferences"""
        best_action = available_actions[0]
        best_value = float('-inf')

        for action in available_actions:
            action_key = f"{context}_{state_feature}_{str(action)}"
            value = self.action_preferences.get(action_key, {}).get('value', 0.0)

            if value > best_value:
                best_value = value
                best_action = action

        return best_action

    def transfer_learning(self, source_context: str, target_context: str):
        """Transfer knowledge between similar contexts"""
        source_experiences = self.experience_buffer.get_experiences_by_context(source_context)
        target_experiences = self.experience_buffer.get_experiences_by_context(target_context)

        # Calculate context similarity
        similarity = self._calculate_context_similarity(source_context, target_context)

        if similarity > self.transfer_threshold:
            # Transfer applicable knowledge
            for exp in source_experiences:
                # Modify context and add to target context
                transferred_exp = exp.copy()
                transferred_exp['context'] = target_context
                # Apply similarity-based reward adjustment
                transferred_exp['reward'] *= similarity
                self.experience_buffer.add_experience(
                    transferred_exp['state'],
                    transferred_exp['action'],
                    transferred_exp['outcome'],
                    transferred_exp['reward'],
                    target_context
                )

    def _calculate_context_similarity(self, ctx1: str, ctx2: str) -> float:
        """Calculate similarity between contexts"""
        if ctx1 == ctx2:
            return 1.0

        # Use object overlap as similarity measure
        ctx1_objects = set(self.context_detector.contexts.get(ctx1, {}).get('objects', []))
        ctx2_objects = set(self.context_detector.contexts.get(ctx2, {}).get('objects', []))

        if not ctx1_objects or not ctx2_objects:
            return 0.0

        intersection = len(ctx1_objects.intersection(ctx2_objects))
        union = len(ctx1_objects.union(ctx2_objects))
        return intersection / union if union > 0 else 0.0

    def _extract_state_feature(self, state: Dict) -> str:
        """Extract relevant state features for learning"""
        features = []

        # Position-based features
        if 'position' in state:
            pos = state['position']
            features.append(f"pos_{int(pos[0])}_{int(pos[1])}")  # Discretize position

        # Affordance-based features
        if 'affordances' in state:
            aff_counts = {k: len(v) for k, v in state['affordances'].items() if v}
            for aff_type, count in aff_counts.items():
                features.append(f"{aff_type}_{count}")

        return "_".join(features) if features else "default"


class SituatedLearningDemo(Node):
    """Complete situated learning demonstration"""

    def __init__(self):
        super().__init__('situated_learning_demo')

        # Initialize situated learning systems
        self.learner = SituatedLearner()
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

        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
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

        # Timer for learning loop
        self.learning_timer = self.create_timer(0.1, self.learning_loop)
        self.action_timer = self.create_timer(0.5, self.action_execution_loop)

        # Robot state
        self.perception_data = {
            'objects': [],
            'obstacles': [],
            'affordances': {}
        }
        self.robot_position = (0.0, 0.0, 0.0)
        self.current_action = None
        self.last_action_outcome = None
        self.learning_state = LearningState.EXPLORING
        self.episode_count = 0
        self.total_reward = 0.0

        self.get_logger().info('Situated Learning Demo initialized')

    def command_callback(self, msg):
        """Process commands to influence learning"""
        command = msg.data.lower()
        self.get_logger().info(f'Received command: {command}')

        if 'explore' in command:
            self.learning_state = LearningState.EXPLORING
            response = "Switching to exploration mode. Learning through environmental interaction."
        elif 'learn' in command:
            self.learning_state = LearningState.LEARNING
            response = "Entering focused learning mode. Building context-dependent knowledge."
        elif 'adapt' in command:
            self.learning_state = LearningState.ADAPTING
            response = "Entering adaptation mode. Adjusting behavior based on experience."
        else:
            response = f"Command '{command}' received. Current learning state: {self.learning_state.value}"

        self.publish_response(response)

    def image_callback(self, msg):
        """Process visual input for object detection"""
        try:
            # Simulate object detection
            objects = self._simulate_object_detection()
            self.perception_data['objects'] = objects
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def lidar_callback(self, msg):
        """Process LiDAR input for spatial perception"""
        obstacles = []
        if msg.ranges:
            for i, range_val in enumerate(msg.ranges):
                if 0 < range_val < 3.0:
                    angle = msg.angle_min + i * msg.angle_increment
                    obstacles.append({
                        'distance': range_val,
                        'angle': angle
                    })
        self.perception_data['obstacles'] = obstacles

    def joint_callback(self, msg):
        """Process joint state for body awareness"""
        pass  # Used for body schema in real implementation

    def odom_callback(self, msg):
        """Process odometry for position tracking"""
        pos = msg.pose.pose.position
        self.robot_position = (pos.x, pos.y, pos.z)

    def _simulate_object_detection(self):
        """Simulate object detection for demonstration"""
        # In different positions, detect different objects
        x, y, _ = self.robot_position
        objects = []

        # Kitchen-like objects near origin
        if abs(x) < 2 and abs(y) < 2:
            objects = [
                {"name": "table", "position": (0.5, 0.5, 0.0), "confidence": 0.9},
                {"name": "chair", "position": (-0.5, 0.5, 0.0), "confidence": 0.8},
                {"name": "cup", "position": (0.6, 0.6, 0.8), "confidence": 0.85}
            ]
        # Living room-like objects in another area
        elif abs(x) > 3:
            objects = [
                {"name": "sofa", "position": (4.0, 1.0, 0.0), "confidence": 0.95},
                {"name": "tv", "position": (4.5, 0.0, 0.5), "confidence": 0.9},
                {"name": "table", "position": (3.5, 0.5, 0.0), "confidence": 0.85}
            ]

        return objects

    def learning_loop(self):
        """Main learning loop"""
        # Update affordances based on perception
        self._update_affordances()

        # Log current state for learning
        current_state = self._get_current_state()

        # If we have a previous action and outcome, learn from it
        if self.current_action and self.last_action_outcome is not None:
            reward = self._calculate_reward(self.current_action, self.last_action_outcome)
            self.learner.learn_from_interaction(
                current_state,
                self.current_action,
                self.last_action_outcome,
                reward
            )
            self.total_reward += reward
            self.last_action_outcome = None  # Reset

    def action_execution_loop(self):
        """Main action execution loop"""
        current_state = self._get_current_state()
        available_actions = self._get_available_actions()

        # Select action based on learning
        action = self.learner.select_action(current_state, available_actions)

        # Execute action
        self.current_action = action
        self.execute_action(action)

        # Update outcome after some delay (would be real outcome in real system)
        self.last_action_outcome = self._simulate_action_outcome(action)

        # Update learning state based on progress
        self._update_learning_state()

        # Log learning progress
        self.episode_count += 1
        if self.episode_count % 50 == 0:  # Log every 50 episodes
            avg_reward = self.total_reward / max(self.episode_count, 1)
            self.get_logger().info(
                f'Learning Progress - Episodes: {self.episode_count}, '
                f'Avg Reward: {avg_reward:.3f}, '
                f'Current State: {self.learning_state.value}, '
                f'Context: {self.learner.context_detector.current_context}'
            )

    def _update_affordances(self):
        """Update affordance information based on perception"""
        # Simple affordance detection based on objects
        affordances = {'graspable': [], 'navigable': [], 'support': []}

        for obj in self.perception_data['objects']:
            obj_name = obj['name'].lower()
            if obj_name in ['cup', 'box', 'bottle']:
                affordances['graspable'].append(obj)
            elif obj_name in ['table', 'chair', 'sofa']:
                affordances['support'].append(obj)

        # Add navigable spaces based on LiDAR
        for obstacle in self.perception_data['obstacles']:
            if obstacle['distance'] > 1.0:  # Clear path
                affordances['navigable'].append(obstacle)

        self.perception_data['affordances'] = affordances

    def _get_current_state(self) -> Dict:
        """Get current embodied state"""
        return {
            'position': self.robot_position,
            'perception': self.perception_data,
            'affordances': self.perception_data['affordances'],
            'velocity': (0.0, 0.0, 0.0)  # Simplified
        }

    def _get_available_actions(self) -> List:
        """Get available actions based on current state"""
        actions = []

        # Movement actions
        actions.extend([
            {'type': 'move', 'direction': 'forward', 'speed': 0.3},
            {'type': 'move', 'direction': 'backward', 'speed': 0.2},
            {'type': 'turn', 'direction': 'left', 'angle': 0.5},
            {'type': 'turn', 'direction': 'right', 'angle': 0.5}
        ])

        # If there are graspable objects, add manipulation actions
        if self.perception_data['affordances']['graspable']:
            actions.append({'type': 'approach', 'target': 'graspable'})

        return actions

    def execute_action(self, action):
        """Execute the selected action"""
        cmd = Twist()

        if action['type'] == 'move':
            if action['direction'] == 'forward':
                cmd.linear.x = action['speed']
            elif action['direction'] == 'backward':
                cmd.linear.x = -action['speed']
        elif action['type'] == 'turn':
            if action['direction'] == 'left':
                cmd.angular.z = action['angle']
            elif action['direction'] == 'right':
                cmd.angular.z = -action['angle']
        elif action['type'] == 'approach':
            cmd.linear.x = 0.2  # Approach speed

        self.cmd_vel_publisher.publish(cmd)

    def _simulate_action_outcome(self, action) -> Dict:
        """Simulate action outcome for learning"""
        # In a real system, this would come from sensors after action execution
        return {
            'success': random.random() > 0.2,  # 80% success rate for demonstration
            'distance_moved': random.uniform(0.1, 0.5),
            'objects_interacted': random.randint(0, 2),
            'time_taken': random.uniform(0.5, 2.0)
        }

    def _calculate_reward(self, action, outcome) -> float:
        """Calculate reward based on action and outcome"""
        reward = 0.0

        # Base reward for taking action
        reward += 0.1

        # Success bonus
        if outcome.get('success', False):
            reward += 0.5

        # Exploration bonus for trying new things
        if action.get('type') == 'approach':
            reward += 0.3  # Bonus for interacting with objects

        # Context-dependent bonuses
        current_context = self.learner.context_detector.current_context
        if current_context == 'kitchen' and action.get('type') == 'approach':
            reward += 0.2  # Extra bonus in kitchen for object interaction

        return max(0.0, min(1.0, reward))  # Clamp between 0 and 1

    def _update_learning_state(self):
        """Update learning state based on progress"""
        # Simple state transition logic
        if self.learning_state == LearningState.EXPLORING:
            # After sufficient exploration, move to learning
            if self.episode_count > 100 and self.total_reward / max(self.episode_count, 1) > 0.3:
                self.learning_state = LearningState.LEARNING
        elif self.learning_state == LearningState.LEARNING:
            # After learning, move to exploitation if performance is good
            if (self.episode_count > 200 and
                self.total_reward / max(self.episode_count, 1) > 0.6):
                self.learning_state = LearningState.EXPLOITING

    def publish_response(self, response_text):
        """Publish AI response to user"""
        response_msg = String()
        response_msg.data = response_text
        self.response_publisher.publish(response_msg)
        self.get_logger().info(f'AI Response: {response_text}')


def main(args=None):
    rclpy.init(args=args)

    demo = SituatedLearningDemo()

    try:
        rclpy.spin(demo)
    except KeyboardInterrupt:
        demo.get_logger().info('Shutting down Situated Learning Demo')
        # Print final statistics
        avg_reward = demo.total_reward / max(demo.episode_count, 1)
        demo.get_logger().info(
            f'Final Statistics - Episodes: {demo.episode_count}, '
            f'Average Reward: {avg_reward:.3f}, '
            f'Final Learning State: {demo.learning_state.value}'
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