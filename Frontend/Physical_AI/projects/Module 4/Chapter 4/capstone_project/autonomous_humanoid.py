#!/usr/bin/env python3
"""
Autonomous Humanoid Controller - Capstone Project

This script integrates all components from the course into a complete
autonomous humanoid system that demonstrates the full Vision-Language-Action
control loop with cognitive reasoning and safe operation.
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
import numpy as np


@dataclass
class SystemState:
    """Represents the overall state of the autonomous humanoid system"""
    perception_data: Dict[str, Any] = None
    cognitive_state: Dict[str, Any] = None
    action_queue: List[Dict[str, Any]] = None
    system_health: Dict[str, bool] = None
    safety_status: str = "nominal"
    task_progress: float = 0.0
    human_interaction_state: Dict[str, Any] = None


class AutonomousHumanoid(Node):
    """
    Main controller for the autonomous humanoid system
    """

    def __init__(self):
        super().__init__('autonomous_humanoid')

        # Initialize all system components
        self.perception_system = PerceptionFusion()
        self.cognitive_engine = CognitiveEngine()
        self.action_executor = ActionExecutor()
        self.safety_system = SafetyMonitor()
        self.human_interaction = HumanInteractionManager()

        # Internal state
        self.system_state = SystemState(
            perception_data={},
            cognitive_state={},
            action_queue=[],
            system_health={},
            safety_status="nominal",
            task_progress=0.0,
            human_interaction_state={}
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
            '/autonomous_humanoid/status',
            10
        )

        self.feedback_publisher = self.create_publisher(
            String,
            '/autonomous_humanoid/feedback',
            10
        )

        self.speech_publisher = self.create_publisher(
            String,
            '/autonomous_humanoid/speech',
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
        self.max_action_queue_size = 10

        self.get_logger().info('Autonomous Humanoid system initialized')

    def voice_command_callback(self, msg: String):
        """Handle incoming voice commands"""
        command = msg.data.strip()
        if command:
            self.get_logger().info(f"Received voice command: '{command}'")

            # Process through human interaction manager
            processed_command = self.human_interaction.process_command(command)

            # Add to cognitive system for processing
            self.system_state.cognitive_state['pending_command'] = processed_command

            feedback_msg = String()
            feedback_msg.data = f"Heard: {command}"
            self.feedback_publisher.publish(feedback_msg)

    def vision_callback(self, msg: Detection2DArray):
        """Handle incoming vision detections"""
        try:
            # Process with perception system
            objects = self.perception_system.process_vision_data(msg)
            self.system_state.perception_data['objects'] = objects

            self.get_logger().debug(f"Updated vision data with {len(objects)} objects")

        except Exception as e:
            self.get_logger().error(f"Error processing vision message: {e}")

    def lidar_callback(self, msg: LaserScan):
        """Handle incoming LiDAR data"""
        try:
            # Process with perception system
            obstacles = self.perception_system.process_lidar_data(msg)
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
            perception_result = self.perception_system.fuse_sensors(
                self.system_state.perception_data
            )

            # 3. Run cognitive engine
            cognitive_result = self.cognitive_engine.process(
                perception_result,
                self.system_state.cognitive_state
            )

            # 4. Check safety
            safety_status = self.safety_system.validate_action(
                cognitive_result,
                self.system_state.perception_data
            )

            if safety_status['safe']:
                # 5. Generate actions if available
                if cognitive_result.get('action_needed', False):
                    action = cognitive_result.get('action', {})
                    if action:
                        # Add to action queue with safety validation
                        if self._is_action_safe(action):
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

                # Provide safety feedback
                feedback_msg = String()
                feedback_msg.data = f"Safety violation: {safety_status.get('reason', 'Unknown')}"
                self.feedback_publisher.publish(feedback_msg)

            # 8. Handle human interaction
            interaction_update = self.human_interaction.update(
                self.system_state.cognitive_state,
                self.system_state.perception_data
            )
            if interaction_update.get('speak'):
                speech_msg = String()
                speech_msg.data = interaction_update['speak']
                self.speech_publisher.publish(speech_msg)

            # 9. Publish system status
            self._publish_system_status()

        except Exception as e:
            self.get_logger().error(f"Error in integration loop: {e}")
            self.system_state.safety_status = "error"
            self._stop_robot()

            feedback_msg = String()
            feedback_msg.data = f"System error: {str(e)}"
            self.feedback_publisher.publish(feedback_msg)

    def _update_system_health(self):
        """Update system health status"""
        # Check all components
        self.system_state.system_health = {
            'perception': self.perception_system.is_operational(),
            'cognition': self.cognitive_engine.is_operational(),
            'action': self.action_executor.is_operational(),
            'safety': self.safety_system.is_operational(),
            'human_interaction': self.human_interaction.is_operational()
        }

    def _is_action_safe(self, action: Dict[str, Any]) -> bool:
        """Check if an action is safe to add to the queue"""
        # In a real implementation, this would do detailed safety checks
        # For demo, just check if queue is not full
        return len(self.system_state.action_queue) < self.max_action_queue_size

    def _execute_queued_actions(self):
        """Execute actions in the queue"""
        while self.system_state.action_queue:
            action = self.system_state.action_queue.pop(0)

            success = self.action_executor.execute_action(action)

            if not success:
                self.get_logger().error(f"Action execution failed: {action}")
                # Add safety stop
                self._stop_robot()
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
            'obstacle_count': len(self.system_state.perception_data.get('obstacles', [])),
            'human_interaction_state': self.system_state.human_interaction_state
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        self.system_status_publisher.publish(status_msg)

    def start_system(self):
        """Start the autonomous humanoid system"""
        self.is_running = True
        self.get_logger().info("Autonomous humanoid system started")

        # Publish startup message
        feedback_msg = String()
        feedback_msg.data = "Autonomous humanoid system is now operational"
        self.feedback_publisher.publish(feedback_msg)

    def stop_system(self):
        """Stop the autonomous humanoid system"""
        self.is_running = False
        self._stop_robot()
        self.get_logger().info("Autonomous humanoid system stopped")

        # Publish shutdown message
        feedback_msg = String()
        feedback_msg.data = "Autonomous humanoid system has shut down"
        self.feedback_publisher.publish(feedback_msg)

    def get_system_state(self) -> Dict[str, Any]:
        """Get the current system state"""
        return {
            'perception_data': self.system_state.perception_data,
            'cognitive_state': self.system_state.cognitive_state,
            'action_queue': self.system_state.action_queue,
            'system_health': self.system_state.system_health,
            'safety_status': self.system_state.safety_status,
            'task_progress': self.system_state.task_progress,
            'human_interaction_state': self.system_state.human_interaction_state
        }


class PerceptionFusion:
    """Handles multimodal perception and sensor fusion"""

    def __init__(self):
        self.objects = []
        self.obstacles = []
        self.robot_pose = None

    def process_vision_data(self, vision_msg: Detection2DArray) -> List[Dict[str, Any]]:
        """Process vision data into object list"""
        objects = []
        for detection in vision_msg.detections:
            if detection.results:
                obj = {
                    'name': detection.results[0].hypothesis.class_id,
                    'confidence': detection.results[0].hypothesis.score,
                    'position': {
                        'x': detection.bbox.center.x,
                        'y': detection.bbox.center.y
                    },
                    'bbox': {
                        'x': detection.bbox.center.x - detection.bbox.size_x / 2,
                        'y': detection.bbox.center.y - detection.bbox.size_y / 2,
                        'width': detection.bbox.size_x,
                        'height': detection.bbox.size_y
                    }
                }
                objects.append(obj)
        return objects

    def process_lidar_data(self, lidar_msg: LaserScan) -> List[Dict[str, float]]:
        """Process LiDAR data into obstacle list"""
        obstacles = []
        for i, range_val in enumerate(lidar_msg.ranges):
            if 0.1 < range_val < 2.0:  # Obstacles within 2 meters
                angle = lidar_msg.angle_min + i * lidar_msg.angle_increment
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                obstacles.append({'x': x, 'y': y, 'distance': range_val})
        return obstacles

    def fuse_sensors(self, perception_data: Dict[str, Any]) -> Dict[str, Any]:
        """Fuse data from all sensors"""
        return {
            'objects': perception_data.get('objects', []),
            'obstacles': perception_data.get('obstacles', []),
            'robot_pose': perception_data.get('robot_pose', {}),
            'joint_states': perception_data.get('joint_states', {}),
            'timestamp': time.time()
        }

    def is_operational(self) -> bool:
        """Check if perception system is operational"""
        return True


class CognitiveEngine:
    """Handles reasoning, planning, and decision making"""

    def __init__(self):
        self.current_task = None
        self.task_history = []
        self.memory = {}  # Short-term memory

    def process(self, perception_data: Dict[str, Any], cognitive_state: Dict[str, Any]) -> Dict[str, Any]:
        """Process perception and cognitive state to generate actions"""
        # Check for new commands
        pending_command = cognitive_state.get('pending_command')
        if pending_command:
            # Create a new task based on the command
            self.current_task = {
                'command': pending_command,
                'status': 'planning',
                'start_time': time.time(),
                'plan': self._generate_plan(pending_command, perception_data)
            }
            cognitive_state['pending_command'] = None

        # Execute current task if it exists
        if self.current_task:
            result = self._execute_task(self.current_task, perception_data)
            return result

        # No active task, return idle state
        return {
            'action_needed': False,
            'action': {},
            'progress': 0.0,
            'task_status': 'idle'
        }

    def _generate_plan(self, command: str, perception_data: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Generate a plan for the given command"""
        # Simple command parsing for demo
        command_lower = command.lower()

        if "find" in command_lower or "locate" in command_lower:
            # Look for objects in the environment
            target_obj = None
            for obj in perception_data.get('objects', []):
                if any(word in obj['name'].lower() for word in ['cube', 'sphere', 'cylinder', 'book']):
                    target_obj = obj
                    break

            if target_obj:
                return [
                    {
                        'type': 'navigate',
                        'target': {'x': target_obj['position']['x'], 'y': target_obj['position']['y']},
                        'description': f"Navigate to {target_obj['name']}"
                    }
                ]
            else:
                return [
                    {
                        'type': 'explore',
                        'description': "Explore environment to find requested object"
                    }
                ]

        elif "move" in command_lower or "go" in command_lower:
            # Simple navigation
            return [
                {
                    'type': 'navigate',
                    'target': {'x': 1.0, 'y': 1.0},  # Default target
                    'description': "Move to target location"
                }
            ]

        else:
            # Default action
            return [
                {
                    'type': 'wait',
                    'duration': 2.0,
                    'description': "Wait for further instructions"
                }
            ]

    def _execute_task(self, task: Dict[str, Any], perception_data: Dict[str, Any]) -> Dict[str, Any]:
        """Execute the current task and return status"""
        if task['status'] == 'planning':
            task['status'] = 'executing'

        # Calculate progress (simplified)
        elapsed = time.time() - task.get('start_time', time.time())
        progress = min(1.0, elapsed / 20.0)  # Assume 20 second max task time

        if progress >= 1.0:
            # Task completed
            task['status'] = 'completed'
            self.task_history.append(task)
            self.current_task = None

            return {
                'action_needed': False,
                'action': {},
                'progress': 1.0,
                'task_status': 'completed'
            }
        else:
            # Return next action in plan
            plan = task.get('plan', [])
            if plan:
                next_action = plan[0]  # Simplified - just return first action
                return {
                    'action_needed': True,
                    'action': next_action,
                    'progress': progress,
                    'task_status': 'executing'
                }
            else:
                return {
                    'action_needed': False,
                    'action': {},
                    'progress': progress,
                    'task_status': 'executing'
                }

    def is_operational(self) -> bool:
        """Check if cognitive engine is operational"""
        return True


class ActionExecutor:
    """Handles action execution and coordination"""

    def __init__(self):
        self.current_action = None
        self.action_history = []

    def execute_action(self, action: Dict[str, Any]) -> bool:
        """Execute an action and return success status"""
        action_type = action.get('type', 'unknown')

        # In a real implementation, this would execute actual robot actions
        # For demo, we'll just simulate the action

        if action_type == 'navigate':
            target = action.get('target', {})
            print(f"Simulating navigation to ({target.get('x', 0)}, {target.get('y', 0)})")
            time.sleep(1.0)  # Simulate execution time
        elif action_type == 'grasp':
            obj = action.get('object', 'unknown')
            print(f"Simulating grasp of {obj}")
            time.sleep(1.0)
        elif action_type == 'speak':
            text = action.get('text', '')
            print(f"Simulating speech: {text}")
            time.sleep(1.0)
        elif action_type == 'explore':
            print("Simulating exploration behavior")
            time.sleep(2.0)
        elif action_type == 'wait':
            duration = action.get('duration', 1.0)
            print(f"Simulating wait for {duration} seconds")
            time.sleep(duration)
        else:
            print(f"Simulating action: {action_type}")
            time.sleep(0.5)

        # Add to history
        self.action_history.append({
            'action': action,
            'timestamp': time.time(),
            'success': True
        })

        return True

    def is_operational(self) -> bool:
        """Check if action executor is operational"""
        return True


class SafetyMonitor:
    """Handles safety validation and monitoring"""

    def __init__(self):
        self.safety_log = []

    def validate_action(self, cognitive_result: Dict[str, Any], perception_data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate if an action is safe to execute"""
        action = cognitive_result.get('action', {})

        # Check for obstacles in path
        obstacles = perception_data.get('obstacles', [])

        if action.get('type') == 'navigate':
            target = action.get('target', {})
            # Check if path to target is clear
            for obstacle in obstacles:
                dist_to_target = ((obstacle['x'] - target.get('x', 0))**2 +
                                (obstacle['y'] - target.get('y', 0))**2)**0.5
                if dist_to_target < 0.5:  # Obstacle too close to target
                    self.safety_log.append({
                        'timestamp': time.time(),
                        'issue': f'Obstacle at ({obstacle["x"]:.2f}, {obstacle["y"]:.2f}) blocks path to target',
                        'action': action
                    })
                    return {
                        'safe': False,
                        'reason': f'Obstacle at ({obstacle["x"]:.2f}, {obstacle["y"]:.2f}) blocks path to target',
                        'severity': 'high'
                    }

        # Check for joint limits (simplified)
        joint_states = perception_data.get('joint_states', {})
        for joint_name, joint_data in joint_states.items():
            if abs(joint_data.get('position', 0)) > 3.0:  # Example limit
                self.safety_log.append({
                    'timestamp': time.time(),
                    'issue': f'Joint {joint_name} exceeds safety limits',
                    'action': action
                })
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

    def is_operational(self) -> bool:
        """Check if safety system is operational"""
        return True


class HumanInteractionManager:
    """Handles human-robot interaction"""

    def __init__(self):
        self.conversation_history = []
        self.user_preferences = {}
        self.trust_level = 0.5  # 0.0 to 1.0

    def process_command(self, command: str) -> str:
        """Process and refine voice command"""
        # In a real implementation, this would do NLP processing
        # For demo, just return the command
        self.conversation_history.append({
            'type': 'command',
            'text': command,
            'timestamp': time.time()
        })
        return command

    def update(self, cognitive_state: Dict[str, Any], perception_data: Dict[str, Any]) -> Dict[str, Any]:
        """Update human interaction state and return any responses"""
        update = {}

        # Check if we should provide feedback
        if cognitive_state.get('task_progress', 0) > 0.5:
            # Task is halfway done, provide status update
            update['speak'] = "I'm making good progress on your request."

        # Update trust based on successful interactions
        self.trust_level = min(1.0, self.trust_level + 0.01)

        return update

    def is_operational(self) -> bool:
        """Check if human interaction system is operational"""
        return True


def main(args=None):
    """
    Main function to run the autonomous humanoid system
    """
    rclpy.init(args=args)

    node = AutonomousHumanoid()

    try:
        node.get_logger().info("Autonomous Humanoid system starting...")
        node.start_system()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Autonomous Humanoid system")
        node.stop_system()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()