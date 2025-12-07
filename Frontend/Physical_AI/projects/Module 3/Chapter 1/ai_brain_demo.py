#!/usr/bin/env python3
"""
AI-Robot Brain Demonstration System

This project demonstrates a complete AI-robot brain that integrates LLMs, VLMs,
and humanoid control systems with attention mechanisms and learning capabilities.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import json
import time
from enum import Enum
from typing import Dict, List, Optional, Any
import threading
import queue


class RobotMode(Enum):
    IDLE = "idle"
    PERCEIVING = "perceiving"
    REASONING = "reasoning"
    PLANNING = "planning"
    ACTING = "acting"
    LEARNING = "learning"
    SAFETY = "safety"


class SafetyMonitor:
    """Monitors safety constraints and triggers emergency responses"""

    def __init__(self):
        self.safety_constraints = {
            'max_speed': 1.0,  # m/s
            'min_obstacle_distance': 0.3,  # meters
            'max_tilt_angle': 0.5,  # radians
            'max_current': 10.0,  # amps (simulated)
        }
        self.emergency_active = False

    def check_safety(self, sensor_data: Dict[str, Any]) -> Dict[str, bool]:
        """Check safety constraints against sensor data"""
        violations = {}

        # Check obstacle distance
        if 'lidar' in sensor_data and sensor_data['lidar']:
            min_dist = min([r for r in sensor_data['lidar'].ranges if r > 0 and r < float('inf')], default=float('inf'))
            violations['obstacle'] = min_dist < self.safety_constraints['min_obstacle_distance']

        # Check IMU data for tilt
        if 'imu' in sensor_data and sensor_data['imu']:
            # Simplified tilt check (in real system, would calculate actual tilt)
            violations['tilt'] = False  # Placeholder

        # Overall safety status
        violations['safe'] = not any(v for k, v in violations.items() if k != 'safe')

        return violations

    def trigger_emergency_stop(self):
        """Trigger emergency stop and safety mode"""
        self.emergency_active = True
        print("EMERGENCY STOP TRIGGERED - Safety system activated")


class ExecutiveController:
    """High-level executive that coordinates cognitive systems"""

    def __init__(self):
        self.current_mode = RobotMode.IDLE
        self.task_queue = queue.Queue()
        self.interrupted = False
        self.safety_monitor = SafetyMonitor()

    def add_task(self, task: Dict[str, Any]):
        """Add task to execution queue"""
        self.task_queue.put(task)

    def interrupt_current_task(self):
        """Interrupt current task execution"""
        self.interrupted = True

    def get_next_task(self) -> Optional[Dict[str, Any]]:
        """Get next task from queue"""
        try:
            return self.task_queue.get_nowait()
        except queue.Empty:
            return None


class AIBrainDemo(Node):
    """Complete AI-robot brain demonstration"""

    def __init__(self):
        super().__init__('ai_brain_demo')

        # Initialize cognitive systems
        self.cv_bridge = CvBridge()
        self.executive = ExecutiveController()
        self.safety_monitor = SafetyMonitor()

        # Subscribe to all sensor inputs
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
        self.status_publisher = self.create_publisher(String, '/robot/status', 10)

        # Timer for main cognitive loop
        self.main_timer = self.create_timer(0.05, self.cognitive_loop)  # 20 Hz
        self.safety_timer = self.create_timer(0.1, self.safety_check)  # 10 Hz

        # Robot state
        self.sensory_data = {
            'image': None,
            'lidar': None,
            'imu': None,
            'odom': None
        }
        self.robot_state = {
            'position': (0.0, 0.0, 0.0),
            'orientation': (0.0, 0.0, 0.0, 1.0),  # quaternion
            'velocity': (0.0, 0.0, 0.0),
            'battery_level': 100.0,
            'temperature': 25.0
        }
        self.current_behavior = "idle"
        self.behavior_history = []
        self.learning_buffer = []

        # AI-robot brain components simulation
        self.memory_traces = []
        self.attention_weights = {'visual': 0.3, 'lidar': 0.3, 'language': 0.4}
        self.confidence_levels = {'perception': 0.8, 'reasoning': 0.7, 'action': 0.9}

        self.get_logger().info('AI-Brain Demonstration System initialized')

    def command_callback(self, msg):
        """Process high-level commands"""
        command_text = msg.data
        self.get_logger().info(f'Received command: {command_text}')

        # Add command processing task
        task = {
            'type': 'process_command',
            'content': command_text,
            'timestamp': time.time()
        }
        self.executive.add_task(task)

        # Simulate AI reasoning about command
        self.reason_about_command(command_text)

    def image_callback(self, msg):
        """Process visual input"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.sensory_data['image'] = cv_image

            # Simulate visual processing
            self.process_visual_input(cv_image)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def lidar_callback(self, msg):
        """Process LiDAR input"""
        self.sensory_data['lidar'] = msg

        # Simulate LiDAR processing
        self.process_lidar_input(msg)

    def imu_callback(self, msg):
        """Process IMU input"""
        self.sensory_data['imu'] = msg

        # Update robot state with IMU data
        self.robot_state['orientation'] = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

    def odom_callback(self, msg):
        """Process odometry input"""
        self.sensory_data['odom'] = msg

        # Update robot state with odometry
        pos = msg.pose.pose.position
        self.robot_state['position'] = (pos.x, pos.y, pos.z)

        vel = msg.twist.twist.linear
        self.robot_state['velocity'] = (vel.x, vel.y, vel.z)

    def process_visual_input(self, image):
        """Process visual input with attention mechanism"""
        # Simulate object detection
        objects = self.simulate_object_detection(image)

        # Update attention based on visual input
        if objects:
            self.attention_weights['visual'] = min(self.attention_weights['visual'] + 0.1, 1.0)
        else:
            self.attention_weights['visual'] = max(self.attention_weights['visual'] - 0.05, 0.1)

        # Store in memory
        self.memory_traces.append({
            'type': 'visual',
            'content': f'Detected {len(objects)} objects',
            'timestamp': time.time(),
            'importance': len(objects) * 0.3
        })

    def process_lidar_input(self, lidar_msg):
        """Process LiDAR input for navigation and safety"""
        # Check for obstacles
        if lidar_msg.ranges:
            min_dist = min([r for r in lidar_msg.ranges if r > 0 and r < float('inf')], default=float('inf'))

            # Update attention based on obstacle detection
            if min_dist < 2.0:
                self.attention_weights['lidar'] = min(self.attention_weights['lidar'] + 0.2, 1.0)
            else:
                self.attention_weights['lidar'] = max(self.attention_weights['lidar'] - 0.05, 0.1)

            # Store in memory
            self.memory_traces.append({
                'type': 'lidar',
                'content': f'Min obstacle distance: {min_dist:.2f}m',
                'timestamp': time.time(),
                'importance': (2.0 - min_dist) * 0.5 if min_dist < 2.0 else 0.0
            })

    def simulate_object_detection(self, image):
        """Simulate object detection in image"""
        # For demonstration, return some objects
        # In real implementation, this would use actual object detection
        return [
            {"name": "person", "confidence": 0.9, "bbox": [100, 100, 200, 200]},
            {"name": "chair", "confidence": 0.8, "bbox": [300, 150, 400, 250]}
        ]

    def reason_about_command(self, command):
        """Simulate AI reasoning about natural language command"""
        self.get_logger().info(f'Reasoning about command: {command}')

        # Simple command interpretation
        if 'move' in command.lower() or 'go' in command.lower():
            action = 'navigation'
        elif 'find' in command.lower() or 'look' in command.lower():
            action = 'perception'
        elif 'stop' in command.lower() or 'halt' in command.lower():
            action = 'stop'
        else:
            action = 'unknown'

        # Update confidence based on command clarity
        confidence = 0.8 if action != 'unknown' else 0.3
        self.confidence_levels['reasoning'] = confidence

        # Store reasoning trace
        self.memory_traces.append({
            'type': 'reasoning',
            'content': f'Command "{command}" interpreted as {action}',
            'timestamp': time.time(),
            'importance': confidence
        })

        response = f"I understand you want me to {action}. I'm {self.get_behavior_confidence(action)} confident in my ability to execute this task."
        self.publish_response(response)

    def get_behavior_confidence(self, behavior):
        """Get confidence level for a specific behavior"""
        if behavior == 'navigation':
            return f"{int(self.confidence_levels['action'] * 100)}%"
        elif behavior == 'perception':
            return f"{int(self.confidence_levels['perception'] * 100)}%"
        else:
            return "moderately"

    def cognitive_loop(self):
        """Main cognitive processing loop"""
        # Check for tasks to execute
        task = self.executive.get_next_task()
        if task:
            self.execute_task(task)

        # Update behavior based on current state
        self.update_behavior()

        # Publish status
        status_msg = String()
        status_msg.data = json.dumps({
            'mode': self.executive.current_mode.value,
            'behavior': self.current_behavior,
            'attention_weights': self.attention_weights,
            'confidence_levels': self.confidence_levels,
            'safety_status': not self.safety_monitor.emergency_active
        })
        self.status_publisher.publish(status_msg)

        # Manage memory traces (keep recent important ones)
        self.memory_traces = [
            trace for trace in self.memory_traces
            if time.time() - trace['timestamp'] < 300  # Keep last 5 minutes
        ]

    def safety_check(self):
        """Regular safety monitoring"""
        safety_violations = self.safety_monitor.check_safety(self.sensory_data)

        if not safety_violations.get('safe', True) or self.safety_monitor.emergency_active:
            # Emergency stop
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd)

            if not self.safety_monitor.emergency_active:
                self.safety_monitor.trigger_emergency_stop()

            self.executive.current_mode = RobotMode.SAFETY
            self.publish_response("Safety system activated. Stopping all movement.")
        else:
            self.executive.current_mode = RobotMode.ACTING

    def execute_task(self, task):
        """Execute a task from the queue"""
        task_type = task['type']
        content = task['content']

        if task_type == 'process_command':
            self.execute_command_task(content)
        elif task_type == 'navigation':
            self.execute_navigation_task(content)
        elif task_type == 'perception':
            self.execute_perception_task(content)

    def execute_command_task(self, command):
        """Execute a command processing task"""
        self.executive.current_mode = RobotMode.REASONING

        # Simulate command execution
        if 'forward' in command.lower() or 'move' in command.lower():
            cmd = Twist()
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd)
            self.current_behavior = 'moving_forward'
        elif 'turn' in command.lower() or 'left' in command.lower() or 'right' in command.lower():
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 if 'left' in command.lower() else -0.5
            self.cmd_vel_publisher.publish(cmd)
            self.current_behavior = 'turning'
        elif 'stop' in command.lower():
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd)
            self.current_behavior = 'stopped'

        # Add to behavior history
        self.behavior_history.append({
            'behavior': self.current_behavior,
            'command': command,
            'timestamp': time.time()
        })

    def execute_navigation_task(self, target):
        """Execute navigation task"""
        self.executive.current_mode = RobotMode.PLANNING
        self.current_behavior = f'navigating_to_{target}'

    def execute_perception_task(self, target):
        """Execute perception task"""
        self.executive.current_mode = RobotMode.PERCEIVING
        self.current_behavior = f'perceiving_{target}'

    def update_behavior(self):
        """Update robot behavior based on current state and sensors"""
        if self.executive.current_mode == RobotMode.SAFETY:
            return  # Safety mode overrides all other behaviors

        # Adjust behavior based on sensor input
        if self.sensory_data['lidar']:
            min_dist = min([r for r in self.sensory_data['lidar'].ranges if r > 0 and r < float('inf')], default=float('inf'))
            if min_dist < 0.5 and self.current_behavior == 'moving_forward':
                # Obstacle detected, adjust behavior
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.3  # Turn to avoid obstacle
                self.cmd_vel_publisher.publish(cmd)
                self.current_behavior = 'avoiding_obstacle'

    def publish_response(self, response_text):
        """Publish AI response to user"""
        response_msg = String()
        response_msg.data = response_text
        self.response_publisher.publish(response_msg)
        self.get_logger().info(f'AI Response: {response_text}')


def main(args=None):
    rclpy.init(args=args)

    demo = AIBrainDemo()

    try:
        rclpy.spin(demo)
    except KeyboardInterrupt:
        demo.get_logger().info('Shutting down AI-Brain Demonstration System')
    finally:
        # Emergency stop before shutdown
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        demo.cmd_vel_publisher.publish(cmd)
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()