#!/usr/bin/env python3
"""
Attention and Learning System for AI-Robot Brain

This script implements attention mechanisms and learning capabilities
for the AI-robot brain architecture, enabling efficient resource allocation
and adaptive behavior.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
import numpy as np
import time
from enum import Enum
from typing import Dict, List, Optional, Tuple


class AttentionMode(Enum):
    EXTERNAL = "external"  # Focus on external stimuli
    INTERNAL = "internal"  # Focus on internal processes
    BALANCED = "balanced"  # Balanced attention


class ResourcePriority(Enum):
    HIGH = 3
    MEDIUM = 2
    LOW = 1


class AttentionSystem:
    """Manages attention allocation across different modalities and processes"""

    def __init__(self):
        self.attention_weights = {
            'visual': 1.0,
            'lidar': 1.0,
            'language': 1.0,
            'motor': 1.0
        }
        self.current_mode = AttentionMode.BALANCED
        self.focus_history = []
        self.attention_decay_rate = 0.95

    def update_attention(self, sensory_inputs: Dict[str, float], task_importance: Dict[str, float]):
        """Update attention weights based on inputs and task importance"""
        # Calculate saliency for each modality
        saliency = {}
        for modality, input_strength in sensory_inputs.items():
            task_relevance = task_importance.get(modality, 0.5)
            saliency[modality] = input_strength * task_relevance

        # Update attention weights using saliency and decay
        for modality in self.attention_weights.keys():
            if modality in saliency:
                self.attention_weights[modality] = (
                    0.7 * self.attention_weights[modality] * self.attention_decay_rate +
                    0.3 * saliency[modality]
                )

        # Normalize weights
        total_weight = sum(self.attention_weights.values())
        if total_weight > 0:
            for modality in self.attention_weights:
                self.attention_weights[modality] /= total_weight

        return self.attention_weights

    def get_resource_allocation(self) -> Dict[str, float]:
        """Get current resource allocation based on attention weights"""
        return self.attention_weights.copy()

    def set_attention_mode(self, mode: AttentionMode):
        """Set attention mode (affects resource allocation)"""
        self.current_mode = mode
        if mode == AttentionMode.EXTERNAL:
            # Prioritize sensory processing
            self.attention_weights['visual'] *= 1.5
            self.attention_weights['lidar'] *= 1.5
        elif mode == AttentionMode.INTERNAL:
            # Prioritize reasoning and planning
            self.attention_weights['language'] *= 1.5
            self.attention_weights['motor'] *= 1.5


class LearningSystem:
    """Implements learning mechanisms for the AI-robot brain"""

    def __init__(self):
        self.experience_buffer = []
        self.skill_library = {}
        self.performance_history = {}
        self.learning_rate = 0.1

    def add_experience(self, state, action, reward, next_state, done=False):
        """Add experience to buffer for learning"""
        experience = {
            'state': state,
            'action': action,
            'reward': reward,
            'next_state': next_state,
            'done': done,
            'timestamp': time.time()
        }
        self.experience_buffer.append(experience)

        # Keep buffer size manageable
        if len(self.experience_buffer) > 1000:
            self.experience_buffer = self.experience_buffer[-500:]

    def update_skill(self, skill_name: str, success: bool, performance: float):
        """Update skill based on performance"""
        if skill_name not in self.skill_library:
            self.skill_library[skill_name] = {
                'success_count': 0,
                'attempt_count': 0,
                'avg_performance': 0.0,
                'last_used': time.time()
            }

        skill = self.skill_library[skill_name]
        skill['attempt_count'] += 1
        if success:
            skill['success_count'] += 1

        # Update average performance with exponential moving average
        skill['avg_performance'] = (
            self.learning_rate * performance +
            (1 - self.learning_rate) * skill['avg_performance']
        )
        skill['last_used'] = time.time()

    def get_skill_success_rate(self, skill_name: str) -> float:
        """Get success rate of a skill"""
        if skill_name not in self.skill_library:
            return 0.0
        skill = self.skill_library[skill_name]
        return skill['success_count'] / max(skill['attempt_count'], 1)

    def recommend_skill(self, task_type: str) -> str:
        """Recommend best skill for a task type"""
        relevant_skills = {k: v for k, v in self.skill_library.items() if task_type in k}
        if not relevant_skills:
            return f"default_{task_type}"

        # Choose skill with highest success rate and recent usage
        best_skill = max(relevant_skills.items(),
                        key=lambda x: x[1]['avg_performance'] * (time.time() - x[1]['last_used']))
        return best_skill[0]

    def learn_from_feedback(self, task, action, feedback):
        """Learn from feedback about task execution"""
        # Simple learning from feedback
        success = feedback.get('success', False)
        performance = feedback.get('performance', 0.5)
        skill_name = f"{task}_{action}"

        self.update_skill(skill_name, success, performance)


class ResourceManager:
    """Manages computational and physical resources"""

    def __init__(self):
        self.resource_limits = {
            'cpu': 0.8,  # 80% CPU limit
            'memory': 0.8,  # 80% memory limit
            'power': 1.0,  # Power limit (normalized)
            'time': 0.1  # Maximum processing time per cycle (seconds)
        }
        self.resource_usage = {
            'cpu': 0.0,
            'memory': 0.0,
            'power': 0.0,
            'time': 0.0
        }

    def allocate_resources(self, priorities: Dict[str, ResourcePriority]) -> Dict[str, float]:
        """Allocate resources based on priorities"""
        allocations = {}
        total_priority = sum(p.value for p in priorities.values())

        for resource, priority in priorities.items():
            allocations[resource] = (priority.value / total_priority) if total_priority > 0 else 0.0

        return allocations

    def monitor_usage(self):
        """Monitor current resource usage"""
        # In a real system, this would interface with system monitors
        # For simulation, we'll return dummy values
        self.resource_usage['cpu'] = np.random.uniform(0.1, 0.5)
        self.resource_usage['memory'] = np.random.uniform(0.2, 0.6)
        self.resource_usage['power'] = np.random.uniform(0.3, 0.7)
        return self.resource_usage.copy()


class AdvancedAIBrain(Node):
    """Advanced AI-robot brain with attention and learning"""

    def __init__(self):
        super().__init__('advanced_ai_brain')

        # Initialize cognitive systems
        self.attention_system = AttentionSystem()
        self.learning_system = LearningSystem()
        self.resource_manager = ResourceManager()
        self.cv_bridge = CvBridge()

        # Subscribe to inputs
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

        # Timer for cognitive loop
        self.timer = self.create_timer(0.1, self.cognitive_loop)

        # Robot state
        self.current_task = None
        self.task_history = []
        self.sensory_buffer = {
            'visual': None,
            'lidar': None,
            'language': None
        }
        self.last_action_time = time.time()

        self.get_logger().info('Advanced AI-Brain with Attention and Learning initialized')

    def command_callback(self, msg):
        """Process high-level commands with learning"""
        command_text = msg.data
        self.get_logger().info(f'Received command: {command_text}')

        # Store command and learn from it
        self.sensory_buffer['language'] = command_text
        self.current_task = command_text

        # Update experience with command
        self.learning_system.add_experience(
            state={'command': command_text, 'sensory': self.sensory_buffer},
            action='interpret_command',
            reward=1.0,  # Initial positive reward for receiving command
            next_state=None
        )

    def image_callback(self, msg):
        """Process visual input with attention"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.sensory_buffer['visual'] = cv_image

            # Calculate visual saliency (simplified)
            saliency = self.calculate_visual_saliency(cv_image)
            self.get_logger().debug(f'Visual saliency: {saliency:.2f}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def lidar_callback(self, msg):
        """Process LiDAR input with attention"""
        self.sensory_buffer['lidar'] = msg
        # Calculate LiDAR-based attention (e.g., obstacle proximity)
        if msg.ranges:
            min_range = min([r for r in msg.ranges if r > 0 and r < float('inf')], default=float('inf'))
            proximity_attention = 1.0 / (min_range + 0.1)  # Higher attention for closer obstacles
            self.get_logger().debug(f'Proximity attention: {proximity_attention:.2f}')

    def calculate_visual_saliency(self, image):
        """Calculate visual saliency for attention system"""
        # Simplified saliency calculation
        if image is not None:
            # Convert to grayscale and calculate variance (as a simple saliency measure)
            gray = np.mean(image, axis=2) if len(image.shape) > 2 else image
            saliency = np.var(gray) / 255.0  # Normalize
            return min(saliency, 1.0)
        return 0.0

    def cognitive_loop(self):
        """Advanced cognitive processing loop with attention and learning"""
        current_time = time.time()

        # Monitor resource usage
        resource_usage = self.resource_manager.monitor_usage()

        # Prepare sensory inputs for attention system
        sensory_inputs = {}
        if self.sensory_buffer['visual'] is not None:
            sensory_inputs['visual'] = self.calculate_visual_saliency(self.sensory_buffer['visual'])
        else:
            sensory_inputs['visual'] = 0.0

        if self.sensory_buffer['lidar'] is not None:
            # Calculate LiDAR attention based on obstacle detection
            lidar_data = self.sensory_buffer['lidar']
            min_range = min([r for r in lidar_data.ranges if r > 0 and r < float('inf')], default=float('inf'))
            sensory_inputs['lidar'] = 1.0 / (min_range + 0.1) if min_range < float('inf') else 0.0
        else:
            sensory_inputs['lidar'] = 0.0

        sensory_inputs['language'] = 1.0 if self.sensory_buffer['language'] else 0.0
        sensory_inputs['motor'] = 0.5  # Default motor attention

        # Task-based importance for attention
        task_importance = {
            'visual': 0.8 if self.current_task and 'look' in self.current_task.lower() else 0.5,
            'lidar': 0.9 if self.current_task and 'avoid' in self.current_task.lower() else 0.5,
            'language': 1.0,  # Always important for command processing
            'motor': 0.7 if self.current_task else 0.3
        }

        # Update attention based on sensory inputs and task importance
        attention_weights = self.attention_system.update_attention(sensory_inputs, task_importance)

        # Log attention allocation
        self.get_logger().debug(f'Attention weights: {attention_weights}')

        # Resource allocation based on attention
        resource_priorities = {
            'perception': ResourcePriority(attention_weights.get('visual', 0.5) * 3),
            'reasoning': ResourcePriority(attention_weights.get('language', 0.5) * 3),
            'action': ResourcePriority(attention_weights.get('motor', 0.5) * 3)
        }

        resource_allocations = self.resource_manager.allocate_resources(resource_priorities)

        # Process current task if available
        if self.current_task and (current_time - self.last_action_time > 1.0):  # Prevent rapid actions
            self.process_task()
            self.last_action_time = current_time

    def process_task(self):
        """Process the current task using learned skills and attention"""
        task = self.current_task.lower()

        if 'navigate' in task or 'go to' in task:
            self.execute_navigation_task(task)
        elif 'find' in task or 'look for' in task:
            self.execute_search_task(task)
        elif 'avoid' in task or 'stop' in task:
            self.execute_avoidance_task(task)
        else:
            self.execute_default_task(task)

    def execute_navigation_task(self, task):
        """Execute navigation task with learning"""
        self.get_logger().info(f'Executing navigation task: {task}')

        # Recommend navigation skill based on learning
        skill = self.learning_system.recommend_skill('navigation')
        self.get_logger().info(f'Using navigation skill: {skill}')

        # Execute navigation (simplified)
        cmd = Twist()
        cmd.linear.x = 0.5
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

        # Learn from execution
        feedback = {
            'success': True,
            'performance': 0.8,
            'execution_time': 2.0
        }
        self.learning_system.learn_from_feedback('navigation', skill, feedback)

        response = f"Executing navigation using skill: {skill}"
        self.publish_response(response)

        # Update experience
        self.learning_system.add_experience(
            state={'task': task, 'skill': skill},
            action='navigate',
            reward=feedback['performance'],
            next_state={'completed': True}
        )

    def execute_search_task(self, task):
        """Execute search task with attention to visual input"""
        self.get_logger().info(f'Executing search task: {task}')

        # Focus attention on visual processing
        self.attention_system.set_attention_mode(AttentionMode.EXTERNAL)

        # For demonstration, assume we found something
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.2  # Slow turn to look around
        self.cmd_vel_publisher.publish(cmd)

        response = "Searching for target object. Focusing visual attention."
        self.publish_response(response)

        # Learn from search
        feedback = {
            'success': True,
            'performance': 0.7,
            'execution_time': 3.0
        }
        self.learning_system.learn_from_feedback('search', 'visual_attention', feedback)

    def execute_avoidance_task(self, task):
        """Execute obstacle avoidance with LiDAR attention"""
        self.get_logger().info(f'Executing avoidance task: {task}')

        # Check LiDAR data for obstacles
        if self.sensory_buffer['lidar']:
            lidar_data = self.sensory_buffer['lidar']
            min_range = min([r for r in lidar_data.ranges if r > 0 and r < float('inf')], default=float('inf'))

            cmd = Twist()
            if min_range < 1.0:  # Obstacle detected
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5 if min_range < 0.5 else 0.3  # Turn more sharply for closer obstacles
            else:
                cmd.linear.x = 0.3  # Continue forward
                cmd.angular.z = 0.0

            self.cmd_vel_publisher.publish(cmd)

        response = f"Avoiding obstacles. Minimum distance: {min_range:.2f}m" if self.sensory_buffer['lidar'] else "No LiDAR data available"
        self.publish_response(response)

    def execute_default_task(self, task):
        """Execute default behavior for unknown tasks"""
        self.get_logger().info(f'Executing default task: {task}')

        response = f"I received the command '{task}' but I'm not sure how to respond optimally. Using default behavior."
        self.publish_response(response)

        # Add negative experience for unknown task
        self.learning_system.add_experience(
            state={'task': task},
            action='default_response',
            reward=0.2,  # Low reward for default response
            next_state={'completed': True}
        )

    def publish_response(self, response_text):
        """Publish AI response to user"""
        response_msg = String()
        response_msg.data = response_text
        self.response_publisher.publish(response_msg)
        self.get_logger().info(f'AI Response: {response_text}')


def main(args=None):
    rclpy.init(args=args)

    brain = AdvancedAIBrain()

    try:
        rclpy.spin(brain)
    except KeyboardInterrupt:
        brain.get_logger().info('Shutting down Advanced AI-Brain')
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