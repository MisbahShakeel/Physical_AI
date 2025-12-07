#!/usr/bin/env python3
"""
Core Embodied AI System for Humanoid Robot

This script implements the core components of an embodied AI system that
integrates body, brain, and environment for situated cognition in humanoid robots.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan, JointState, Imu
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import time
from enum import Enum
from typing import Dict, List, Tuple, Optional
import math


class EmbodiedState(Enum):
    IDLE = "idle"
    PERCEIVING = "perceiving"
    ACTING = "acting"
    LEARNING = "learning"
    ADAPTING = "adapting"


class BodySchema:
    """Represents the robot's body schema and self-awareness"""

    def __init__(self):
        self.joint_positions = {}
        self.joint_velocities = {}
        self.end_effector_positions = {}
        self.body_limits = {
            'arm_reach': 1.0,  # meters
            'leg_span': 0.8,   # meters
            'torso_height': 1.5  # meters
        }
        self.center_of_mass = (0.0, 0.0, 0.8)  # meters from ground

    def update_joint_state(self, joint_state: JointState):
        """Update body schema with current joint positions"""
        for i, name in enumerate(joint_state.name):
            if i < len(joint_state.position):
                self.joint_positions[name] = joint_state.position[i]
            if i < len(joint_state.velocity):
                self.joint_velocities[name] = joint_state.velocity[i]

    def get_reachability(self, target_position: Tuple[float, float, float]) -> bool:
        """Check if target is reachable given body constraints"""
        # Calculate distance from current position to target
        # Simplified reachability check
        current_pos = self.center_of_mass
        distance = math.sqrt(
            (target_position[0] - current_pos[0])**2 +
            (target_position[1] - current_pos[1])**2 +
            (target_position[2] - current_pos[2])**2
        )
        return distance <= self.body_limits['arm_reach']

    def get_balance_state(self) -> Dict[str, float]:
        """Get current balance and stability state"""
        # Simplified balance calculation
        return {
            'center_of_mass_height': self.center_of_mass[2],
            'stability': 0.8,  # Placeholder stability measure
            'tilt_angle': 0.0  # Current tilt angle
        }


class AffordanceDetector:
    """Detects and classifies environmental affordances"""

    def __init__(self):
        self.affordances = {
            'surface': [],      # Support surfaces (tables, chairs)
            'opening': [],      # Passages, doors
            'graspable': [],    # Graspable objects
            'navigable': []     # Navigable paths
        }
        self.affordance_thresholds = {
            'surface_height': (0.2, 1.2),  # Valid surface heights
            'opening_width': 0.5,          # Minimum opening width
            'graspable_size': (0.05, 0.3), # Graspable object size range
            'navigable_clearance': 0.3     # Minimum clearance for navigation
        }

    def detect_affordances(self, perception_data: Dict) -> Dict[str, List]:
        """Detect affordances from perception data"""
        detected = {'surface': [], 'opening': [], 'graspable': [], 'navigable': []}

        # Process visual data for affordances
        if 'objects' in perception_data:
            for obj in perception_data['objects']:
                obj_type = obj.get('name', 'unknown')
                position = obj.get('position', (0, 0, 0))
                size = obj.get('size', (0.1, 0.1, 0.1))

                # Classify based on affordance potential
                if self._is_graspable(obj_type, size):
                    detected['graspable'].append({
                        'position': position,
                        'object': obj,
                        'size': size
                    })
                elif self._is_surface(obj_type, position[2]):
                    detected['surface'].append({
                        'position': position,
                        'type': obj_type
                    })

        # Process spatial data for navigation affordances
        if 'obstacles' in perception_data:
            detected['navigable'] = self._analyze_navigable_paths(perception_data['obstacles'])

        return detected

    def _is_graspable(self, obj_type: str, size: Tuple[float, float, float]) -> bool:
        """Determine if an object is graspable"""
        # Check size constraints
        avg_size = sum(size) / len(size)
        min_size, max_size = self.affordance_thresholds['graspable_size']

        # Check object type (some objects are inherently graspable)
        graspable_types = ['box', 'cylinder', 'ball', 'cup', 'bottle']

        return (min_size <= avg_size <= max_size) or (obj_type.lower() in graspable_types)

    def _is_surface(self, obj_type: str, height: float) -> bool:
        """Determine if an object provides a support surface"""
        min_height, max_height = self.affordance_thresholds['surface_height']
        surface_types = ['table', 'chair', 'desk', 'platform']

        return (min_height <= height <= max_height) or (obj_type.lower() in surface_types)

    def _analyze_navigable_paths(self, obstacles: List) -> List:
        """Analyze obstacles to find navigable paths"""
        navigable_paths = []

        # Simplified path analysis
        # In a real system, this would use path planning algorithms
        for obstacle in obstacles:
            if obstacle.get('distance', float('inf')) > self.affordance_thresholds['navigable_clearance']:
                navigable_paths.append(obstacle)

        return navigable_paths


class SensorimotorLearner:
    """Implements sensorimotor learning for embodied adaptation"""

    def __init__(self):
        self.experience_buffer = []
        self.action_outcomes = {}  # Maps actions to outcomes
        self.prediction_models = {}  # Sensorimotor prediction models
        self.learning_rate = 0.1
        self.exploration_rate = 0.3

    def record_experience(self, state, action, outcome, reward):
        """Record sensorimotor experience"""
        experience = {
            'state': state,
            'action': action,
            'outcome': outcome,
            'reward': reward,
            'timestamp': time.time()
        }
        self.experience_buffer.append(experience)

        # Update action-outcome associations
        action_key = str(action)
        if action_key not in self.action_outcomes:
            self.action_outcomes[action_key] = {'outcomes': [], 'rewards': []}

        self.action_outcomes[action_key]['outcomes'].append(outcome)
        self.action_outcomes[action_key]['rewards'].append(reward)

        # Keep buffer size manageable
        if len(self.experience_buffer) > 1000:
            self.experience_buffer = self.experience_buffer[-500:]

    def predict_outcome(self, current_state, action) -> Dict:
        """Predict likely outcome of an action in current state"""
        action_key = str(action)

        if action_key in self.action_outcomes:
            outcomes = self.action_outcomes[action_key]['outcomes']
            rewards = self.action_outcomes[action_key]['rewards']

            # Simple prediction based on past outcomes
            if outcomes:
                avg_outcome = np.mean(outcomes, axis=0) if outcomes else np.zeros(3)
                avg_reward = np.mean(rewards) if rewards else 0.0

                return {
                    'predicted_outcome': avg_outcome,
                    'expected_reward': avg_reward,
                    'confidence': min(len(outcomes) / 10.0, 1.0)  # Confidence based on experience
                }

        # Default prediction for unknown actions
        return {
            'predicted_outcome': np.zeros(3),
            'expected_reward': 0.1,
            'confidence': 0.1
        }

    def select_action(self, current_state, available_actions) -> Tuple:
        """Select action using exploration-exploitation balance"""
        if np.random.random() < self.exploration_rate:
            # Explore: random action
            return available_actions[np.random.randint(len(available_actions))]
        else:
            # Exploit: best known action
            best_action = None
            best_expected_reward = float('-inf')

            for action in available_actions:
                prediction = self.predict_outcome(current_state, action)
                if prediction['expected_reward'] > best_expected_reward:
                    best_expected_reward = prediction['expected_reward']
                    best_action = action

            return best_action if best_action is not None else available_actions[0]

    def update_prediction_model(self, state, action, outcome):
        """Update internal prediction models"""
        # In a real system, this would update neural networks or other models
        # For demonstration, we'll use simple statistical models
        pass


class EmbodiedAIBrain(Node):
    """Main embodied AI brain node"""

    def __init__(self):
        super().__init__('embodied_ai_brain')

        # Initialize embodied systems
        self.body_schema = BodySchema()
        self.affordance_detector = AffordanceDetector()
        self.sensorimotor_learner = SensorimotorLearner()
        self.cv_bridge = CvBridge()

        # Subscribe to all necessary topics
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

        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/humanoid/imu/data',
            self.imu_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.response_publisher = self.create_publisher(String, '/robot/response', 10)

        # Timer for embodied cognitive loop
        self.timer = self.create_timer(0.05, self.embodied_cognitive_loop)  # 20 Hz

        # Robot state
        self.perception_data = {
            'objects': [],
            'obstacles': [],
            'environment': {}
        }
        self.current_state = EmbodiedState.IDLE
        self.intention = None
        self.situated_context = {}
        self.last_action_time = time.time()

        self.get_logger().info('Embodied AI Brain initialized')

    def command_callback(self, msg):
        """Process high-level commands in embodied context"""
        command_text = msg.data
        self.get_logger().info(f'Received embodied command: {command_text}')

        # Update situated context based on command
        self.intention = command_text
        self.current_state = EmbodiedState.PERCEIVING

        # Store as experience
        self.sensorimotor_learner.record_experience(
            state=self._get_current_state(),
            action={'type': 'receive_command', 'content': command_text},
            outcome={'context_updated': True},
            reward=0.5
        )

    def image_callback(self, msg):
        """Process visual input for affordance detection"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Simulate object detection
            objects = self._simulate_object_detection(cv_image)
            self.perception_data['objects'] = objects

            # Detect affordances from visual data
            affordances = self.affordance_detector.detect_affordances(self.perception_data)
            self.perception_data['affordances'] = affordances

            self.get_logger().debug(f'Detected {len(objects)} objects, {len(affordances["graspable"])} graspable items')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def lidar_callback(self, msg):
        """Process spatial input for navigation affordances"""
        # Process LiDAR data for obstacle detection
        obstacles = []
        if msg.ranges:
            for i, range_val in enumerate(msg.ranges):
                if 0 < range_val < 3.0:  # Valid range
                    angle = msg.angle_min + i * msg.angle_increment
                    obstacles.append({
                        'distance': range_val,
                        'angle': angle,
                        'x': range_val * math.cos(angle),
                        'y': range_val * math.sin(angle)
                    })

        self.perception_data['obstacles'] = obstacles

    def joint_state_callback(self, msg):
        """Update body schema with joint information"""
        self.body_schema.update_joint_state(msg)

    def imu_callback(self, msg):
        """Process IMU data for balance and orientation"""
        # Update body schema with orientation
        self.body_schema.center_of_mass = (
            self.body_schema.center_of_mass[0],
            self.body_schema.center_of_mass[1],
            msg.linear_acceleration.z  # Simplified
        )

    def odom_callback(self, msg):
        """Process odometry for spatial context"""
        pos = msg.pose.pose.position
        self.situated_context['position'] = (pos.x, pos.y, pos.z)

    def _simulate_object_detection(self, image):
        """Simulate object detection (in real system, use actual detection)"""
        # For demonstration, return some objects
        return [
            {"name": "red_box", "position": (1.0, 0.5, 0.0), "size": (0.2, 0.2, 0.2), "confidence": 0.9},
            {"name": "table", "position": (0.0, 1.0, 0.0), "size": (1.0, 0.8, 0.8), "confidence": 0.85}
        ]

    def _get_current_state(self) -> Dict:
        """Get current embodied state"""
        return {
            'perception': self.perception_data,
            'body_schema': self.body_schema.joint_positions,
            'position': self.situated_context.get('position', (0, 0, 0)),
            'balance': self.body_schema.get_balance_state(),
            'affordances': self.perception_data.get('affordances', {})
        }

    def embodied_cognitive_loop(self):
        """Main embodied cognitive processing loop"""
        current_time = time.time()

        if self.current_state == EmbodiedState.PERCEIVING:
            # Process perception and update context
            self._process_situated_perception()
            self.current_state = EmbodiedState.ACTING

        elif self.current_state == EmbodiedState.ACTING:
            # Execute action based on intention and affordances
            if self.intention and current_time - self.last_action_time > 1.0:
                self._execute_intention()
                self.last_action_time = current_time

        elif self.current_state == EmbodiedState.LEARNING:
            # Update learning systems based on outcomes
            self._update_learning_systems()

    def _process_situated_perception(self):
        """Process perception in environmental context"""
        # Update situated context with current affordances
        affordances = self.perception_data.get('affordances', {})

        self.situated_context['available_affordances'] = {
            'graspable_objects': len(affordances.get('graspable', [])),
            'navigable_paths': len(affordances.get('navigable', [])),
            'support_surfaces': len(affordances.get('surface', []))
        }

        # Log perception for learning
        self.sensorimotor_learner.record_experience(
            state=self._get_current_state(),
            action={'type': 'perceive'},
            outcome={'context_enriched': True},
            reward=0.2
        )

    def _execute_intention(self):
        """Execute current intention based on affordances and body schema"""
        if not self.intention:
            return

        intention_lower = self.intention.lower()

        if 'approach' in intention_lower or 'go to' in intention_lower:
            self._execute_approach_intention()
        elif 'grasp' in intention_lower or 'pick up' in intention_lower:
            self._execute_grasp_intention()
        elif 'navigate' in intention_lower:
            self._execute_navigation_intention()
        else:
            self._execute_exploration_intention()

    def _execute_approach_intention(self):
        """Execute approach intention using affordance information"""
        graspable_objects = self.perception_data.get('affordances', {}).get('graspable', [])

        if graspable_objects:
            target = graspable_objects[0]  # First available object
            position = target['position']

            # Move toward target
            cmd = Twist()
            cmd.linear.x = 0.3  # Move forward
            cmd.angular.z = 0.0

            # Check reachability
            if self.body_schema.get_reachability(position):
                cmd.linear.x = 0.1  # Slow approach when near
                self.get_logger().info(f'Approaching reachable object at {position}')
            else:
                self.get_logger().info(f'Moving toward object at {position}')

            self.cmd_vel_publisher.publish(cmd)

            # Record experience
            self.sensorimotor_learner.record_experience(
                state=self._get_current_state(),
                action={'type': 'approach', 'target': position},
                outcome={'approaching': True, 'target': position},
                reward=0.7
            )

    def _execute_grasp_intention(self):
        """Execute grasp intention using reachability and affordance information"""
        graspable_objects = self.perception_data.get('affordances', {}).get('graspable', [])

        if graspable_objects:
            target = graspable_objects[0]
            position = target['position']

            if self.body_schema.get_reachability(position):
                # Grasp action (simulated)
                self.get_logger().info(f'Attempting to grasp object at {position}')

                # Record successful grasp attempt
                self.sensorimotor_learner.record_experience(
                    state=self._get_current_state(),
                    action={'type': 'grasp', 'target': position},
                    outcome={'grasping_attempted': True, 'success': True},
                    reward=0.9
                )
            else:
                # Need to approach first
                self.get_logger().info(f'Object at {position} not reachable, need to approach first')
                self.intention = f'approach {target.get("object", {}).get("name", "object")}'
        else:
            self.get_logger().info('No graspable objects detected')

    def _execute_navigation_intention(self):
        """Execute navigation intention using spatial affordances"""
        navigable_paths = self.perception_data.get('affordances', {}).get('navigable', [])

        if navigable_paths:
            # Choose first available path
            path = navigable_paths[0]

            cmd = Twist()
            cmd.linear.x = 0.4  # Navigate forward
            cmd.angular.z = 0.0

            self.cmd_vel_publisher.publish(cmd)
            self.get_logger().info(f'Navigating using path: {path}')

            # Record navigation experience
            self.sensorimotor_learner.record_experience(
                state=self._get_current_state(),
                action={'type': 'navigate', 'path': path},
                outcome={'navigating': True},
                reward=0.6
            )
        else:
            self.get_logger().info('No clear navigable paths detected')

    def _execute_exploration_intention(self):
        """Execute exploration using sensorimotor learning"""
        # Use learned exploration patterns
        available_actions = [
            {'type': 'turn', 'direction': 'left', 'amount': 0.5},
            {'type': 'turn', 'direction': 'right', 'amount': 0.5},
            {'type': 'move', 'direction': 'forward', 'distance': 0.5}
        ]

        selected_action = self.sensorimotor_learner.select_action(
            self._get_current_state(),
            available_actions
        )

        # Execute selected action
        cmd = Twist()
        if selected_action['type'] == 'turn':
            cmd.angular.z = selected_action['amount'] if selected_action['direction'] == 'left' else -selected_action['amount']
        elif selected_action['type'] == 'move':
            cmd.linear.x = selected_action['distance']

        self.cmd_vel_publisher.publish(cmd)

        # Record exploration experience
        self.sensorimotor_learner.record_experience(
            state=self._get_current_state(),
            action=selected_action,
            outcome={'exploring': True},
            reward=0.3
        )

    def _update_learning_systems(self):
        """Update learning systems based on experiences"""
        # This would update internal models in a real implementation
        pass


def main(args=None):
    rclpy.init(args=args)

    brain = EmbodiedAIBrain()

    try:
        rclpy.spin(brain)
    except KeyboardInterrupt:
        brain.get_logger().info('Shutting down Embodied AI Brain')
    finally:
        # Stop the robot before shutting down
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        brain.cmd_vel_publisher.publish(cmd)
        brain.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()