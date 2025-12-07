#!/usr/bin/env python3
"""
Capstone Integrator for Autonomous Humanoid System

This script integrates all components from previous modules into a cohesive
autonomous humanoid system that combines perception, reasoning, and action.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image, LaserScan, JointState
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray
import json
import time
import threading
from typing import Dict, List, Any, Optional
import logging
from dataclasses import dataclass


@dataclass
class SystemState:
    """Represents the overall state of the autonomous humanoid system"""
    perception_data: Dict[str, Any] = None
    cognitive_state: Dict[str, Any] = None
    action_queue: List[Dict[str, Any]] = None
    system_health: Dict[str, bool] = None
    safety_status: str = "nominal"
    task_progress: float = 0.0


class CapstoneIntegratorNode(Node):
    """
    ROS 2 node that integrates all components for the autonomous humanoid system
    """

    def __init__(self):
        super().__init__('capstone_integrator')

        # Initialize system components (using mock implementations for demo)
        self.perception_processor = MockPerceptionProcessor()
        self.cognitive_system = MockCognitiveSystem()
        self.action_executor = MockActionExecutor()
        self.safety_monitor = MockSafetyMonitor()

        # Internal state
        self.system_state = SystemState(
            perception_data={},
            cognitive_state={},
            action_queue=[],
            system_health={},
            safety_status="nominal",
            task_progress=0.0
        )

        self.execution_thread = None
        self.is_running = False
        self.main_loop_rate = 10  # Hz

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.system_status_publisher = self.create_publisher(
            String,
            '/capstone/status',
            10
        )

        self.feedback_publisher = self.create_publisher(
            String,
            '/capstone/feedback',
            10
        )

        # Subscribers
        self.voice_command_subscription = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10
        )

        self.vision_subscription = self.create_subscription(
            Detection2DArray,
            '/vla/detections',
            self.vision_callback,
            10
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for main integration loop
        self.integration_timer = self.create_timer(
            1.0 / self.main_loop_rate,
            self.integration_callback
        )

        # System parameters
        self.safety_threshold = 0.8
        self.task_timeout = 60.0  # seconds

        self.get_logger().info('Capstone Integrator node initialized')

    def voice_command_callback(self, msg: String):
        """Handle incoming voice commands"""
        command = msg.data.strip()
        if command:
            self.get_logger().info(f"Received voice command: '{command}'")

            # Add to cognitive system for processing
            self.system_state.cognitive_state['pending_command'] = command

            feedback_msg = String()
            feedback_msg.data = f"Received: {command}"
            self.feedback_publisher.publish(feedback_msg)

    def vision_callback(self, msg: Detection2DArray):
        """Handle incoming vision detections"""
        try:
            # Convert detections to internal format
            objects = []
            for detection in msg.detections:
                if detection.results:
                    obj = {
                        'name': detection.results[0].hypothesis.class_id,
                        'confidence': detection.results[0].hypothesis.score,
                        'position': {
                            'x': detection.bbox.center.x,
                            'y': detection.bbox.center.y
                        }
                    }
                    objects.append(obj)

            self.system_state.perception_data['objects'] = objects
            self.get_logger().debug(f"Updated vision data with {len(objects)} objects")

        except Exception as e:
            self.get_logger().error(f"Error processing vision message: {e}")

    def lidar_callback(self, msg: LaserScan):
        """Handle incoming LiDAR data"""
        try:
            # Process LiDAR data for obstacles
            obstacles = []
            for i, range_val in enumerate(msg.ranges):
                if 0.1 < range_val < 2.0:  # Obstacles within 2 meters
                    angle = msg.angle_min + i * msg.angle_increment
                    x = range_val * __import__('math').cos(angle)
                    y = range_val * __import__('math').sin(angle)
                    obstacles.append({'x': x, 'y': y, 'distance': range_val})

            self.system_state.perception_data['obstacles'] = obstacles
            self.get_logger().debug(f"Detected {len(obstacles)} obstacles")

        except Exception as e:
            self.get_logger().error(f"Error processing LiDAR message: {e}")

    def odom_callback(self, msg: Odometry):
        """Handle odometry data"""
        self.system_state.perception_data['robot_pose'] = {
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            }
        }

    def joint_state_callback(self, msg: JointState):
        """Handle joint state data"""
        joint_data = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                joint_data[name] = {
                    'position': msg.position[i],
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                }

        self.system_state.perception_data['joint_states'] = joint_data

    def integration_callback(self):
        """Main integration callback - runs the complete system loop"""
        try:
            # 1. Update system health
            self._update_system_health()

            # 2. Process perception data
            perception_result = self.perception_processor.process_data(
                self.system_state.perception_data
            )

            # 3. Run cognitive system
            cognitive_result = self.cognitive_system.process(
                perception_result,
                self.system_state.cognitive_state
            )

            # 4. Check safety
            safety_status = self.safety_monitor.check_safety(
                cognitive_result,
                self.system_state.perception_data
            )

            if safety_status['safe']:
                # 5. Generate actions if available
                if cognitive_result.get('action_needed', False):
                    action = cognitive_result.get('action', {})
                    if action:
                        self.system_state.action_queue.append(action)

                # 6. Execute actions
                self._execute_queued_actions()

                # 7. Update task progress
                self.system_state.task_progress = cognitive_result.get('progress', 0.0)

                self.system_state.safety_status = "nominal"
            else:
                self.get_logger().warn(f"Safety violation detected: {safety_status.get('reason', 'Unknown')}")
                self.system_state.safety_status = "violation"
                # Stop robot for safety
                self._stop_robot()

            # 8. Publish system status
            self._publish_system_status()

        except Exception as e:
            self.get_logger().error(f"Error in integration loop: {e}")
            self.system_state.safety_status = "error"
            self._stop_robot()

    def _update_system_health(self):
        """Update system health status"""
        # In a real system, this would check all components
        self.system_state.system_health = {
            'perception': True,
            'cognition': True,
            'action': True,
            'communication': True,
            'safety': True
        }

    def _execute_queued_actions(self):
        """Execute actions in the queue"""
        while self.system_state.action_queue:
            action = self.system_state.action_queue.pop(0)

            success = self.action_executor.execute_action(action)

            if not success:
                self.get_logger().error(f"Action execution failed: {action}")
                break

    def _stop_robot(self):
        """Emergency stop for safety"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

    def _publish_system_status(self):
        """Publish overall system status"""
        status = {
            'safety_status': self.system_state.safety_status,
            'task_progress': self.system_state.task_progress,
            'action_queue_size': len(self.system_state.action_queue),
            'system_health': self.system_state.system_health,
            'object_count': len(self.system_state.perception_data.get('objects', [])),
            'obstacle_count': len(self.system_state.perception_data.get('obstacles', []))
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        self.system_status_publisher.publish(status_msg)

    def start_system(self):
        """Start the autonomous humanoid system"""
        self.is_running = True
        self.get_logger().info("Autonomous humanoid system started")

    def stop_system(self):
        """Stop the autonomous humanoid system"""
        self.is_running = False
        self._stop_robot()
        self.get_logger().info("Autonomous humanoid system stopped")

    def get_system_state(self) -> Dict[str, Any]:
        """Get the current system state"""
        return {
            'perception_data': self.system_state.perception_data,
            'cognitive_state': self.system_state.cognitive_state,
            'action_queue': self.system_state.action_queue,
            'system_health': self.system_state.system_health,
            'safety_status': self.system_state.safety_status,
            'task_progress': self.system_state.task_progress
        }


class MockPerceptionProcessor:
    """
    Mock perception processor for demonstration
    In a real implementation, this would process actual sensor data
    """

    def __init__(self):
        pass

    def process_data(self, raw_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Process raw sensor data into meaningful perception results
        """
        # In a real implementation, this would run complex perception algorithms
        # For demo, return simplified results
        return {
            'objects': raw_data.get('objects', []),
            'obstacles': raw_data.get('obstacles', []),
            'robot_pose': raw_data.get('robot_pose', {}),
            'joint_states': raw_data.get('joint_states', {}),
            'timestamp': time.time()
        }


class MockCognitiveSystem:
    """
    Mock cognitive system for demonstration
    In a real implementation, this would implement complex reasoning
    """

    def __init__(self):
        self.current_task = None
        self.task_history = []

    def process(self, perception_data: Dict[str, Any], cognitive_state: Dict[str, Any]) -> Dict[str, Any]:
        """
        Process perception data and cognitive state to generate actions
        """
        # Check for new commands
        pending_command = cognitive_state.get('pending_command')
        if pending_command:
            # Process the command and create a task
            self.current_task = {
                'command': pending_command,
                'status': 'in_progress',
                'start_time': time.time()
            }
            cognitive_state['pending_command'] = None

        # Simulate task progress
        if self.current_task:
            elapsed = time.time() - self.current_task.get('start_time', time.time())
            progress = min(1.0, elapsed / 10.0)  # Simulate 10 second task

            result = {
                'action_needed': True,
                'action': {
                    'type': 'navigate',
                    'target': {'x': 1.0, 'y': 1.0},
                    'description': 'Moving to target location'
                },
                'progress': progress,
                'task_status': 'in_progress' if progress < 1.0 else 'completed'
            }

            if progress >= 1.0:
                # Task completed
                self.task_history.append(self.current_task)
                self.current_task = None
        else:
            # No active task, wait
            result = {
                'action_needed': False,
                'action': {},
                'progress': 0.0,
                'task_status': 'idle'
            }

        return result


class MockActionExecutor:
    """
    Mock action executor for demonstration
    In a real implementation, this would execute actual robot actions
    """

    def __init__(self):
        pass

    def execute_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute an action and return success status
        """
        # Simulate action execution
        action_type = action.get('type', 'unknown')
        self.get_logger = lambda: print  # Mock logger for demo

        if action_type == 'navigate':
            target = action.get('target', {})
            print(f"Simulating navigation to ({target.get('x', 0)}, {target.get('y', 0)})")
            time.sleep(0.5)  # Simulate execution time
        elif action_type == 'grasp':
            obj = action.get('object', 'unknown')
            print(f"Simulating grasp of {obj}")
            time.sleep(0.5)
        elif action_type == 'speak':
            text = action.get('text', '')
            print(f"Simulating speech: {text}")
            time.sleep(0.5)
        else:
            print(f"Simulating action: {action_type}")
            time.sleep(0.5)

        # All actions succeed in the mock implementation
        return True


class MockSafetyMonitor:
    """
    Mock safety monitor for demonstration
    In a real implementation, this would implement comprehensive safety checks
    """

    def __init__(self):
        pass

    def check_safety(self, cognitive_result: Dict[str, Any], perception_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Check if the system is in a safe state
        """
        # Check for obstacles in path
        obstacles = perception_data.get('obstacles', [])
        action = cognitive_result.get('action', {})

        if action.get('type') == 'navigate':
            target = action.get('target', {})
            # Check if path to target is clear
            for obstacle in obstacles:
                dist_to_target = ((obstacle['x'] - target.get('x', 0))**2 +
                                (obstacle['y'] - target.get('y', 0))**2)**0.5
                if dist_to_target < 0.5:  # Obstacle too close to target
                    return {
                        'safe': False,
                        'reason': f'Obstacle at ({obstacle["x"]:.2f}, {obstacle["y"]:.2f}) blocks path to target',
                        'severity': 'high'
                    }

        # Check for joint limits (simplified)
        joint_states = perception_data.get('joint_states', {})
        for joint_name, joint_data in joint_states.items():
            if abs(joint_data.get('position', 0)) > 3.0:  # Example limit
                return {
                    'safe': False,
                    'reason': f'Joint {joint_name} exceeds safety limits',
                    'severity': 'medium'
                }

        # All safety checks passed
        return {
            'safe': True,
            'reason': 'All safety checks passed',
            'severity': 'none'
        }


def main(args=None):
    """
    Main function to run the capstone integrator
    """
    rclpy.init(args=args)

    node = CapstoneIntegratorNode()

    try:
        node.get_logger().info("Capstone Integrator starting...")
        node.start_system()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Capstone Integrator")
        node.stop_system()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()