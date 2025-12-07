#!/usr/bin/env python3
"""
LLM Action Sequencer ROS 2 Node

This script implements a ROS 2 node that interprets LLM plans and sequences
basic ROS 2 actions like movement, navigation, and manipulation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import json
import time
from typing import Dict, List, Any, Optional
import logging
from threading import Lock


class LLMActionSequencer(Node):
    """
    ROS 2 node that sequences actions based on LLM-generated plans
    """

    def __init__(self):
        super().__init__('llm_action_sequencer')

        # Internal state
        self.current_plan = None
        self.plan_index = 0
        self.is_executing = False
        self.robot_pose = None
        self.joint_states = None
        self.execution_lock = Lock()

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.status_publisher = self.create_publisher(
            String,
            '/llm_status',
            10
        )

        self.feedback_publisher = self.create_publisher(
            String,
            '/llm_feedback',
            10
        )

        # Subscribers
        self.plan_subscription = self.create_subscription(
            String,
            '/llm_plan',
            self.plan_callback,
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

        # Action clients (for more complex tasks)
        # These would be initialized for specific actions like navigation, manipulation
        self.nav_to_pose_client = None  # Would be initialized for navigation tasks
        self.follow_joint_trajectory_client = None  # Would be initialized for manipulation

        # Timer for plan execution
        self.execution_timer = self.create_timer(
            0.1,  # 10 Hz
            self.execution_callback
        )

        # Action execution parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        self.action_timeout = 10.0  # seconds

        self.get_logger().info('LLM Action Sequencer node initialized')

    def plan_callback(self, msg: String):
        """
        Handle incoming LLM plans
        """
        try:
            plan_data = json.loads(msg.data)
            plan = plan_data.get('actions', [])
            command = plan_data.get('command', 'unknown')

            self.get_logger().info(f"Received plan for command: '{command}' with {len(plan)} actions")

            # Acquire lock to update plan safely
            with self.execution_lock:
                self.current_plan = plan
                self.plan_index = 0
                self.is_executing = True

            # Publish status
            status_msg = String()
            status_msg.data = f"RECEIVED_PLAN: {command}"
            self.status_publisher.publish(status_msg)

        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in plan message: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing plan: {e}")

    def odom_callback(self, msg: Odometry):
        """
        Update robot pose from odometry
        """
        self.robot_pose = msg.pose.pose

    def joint_state_callback(self, msg: JointState):
        """
        Update joint states
        """
        self.joint_states = msg

    def execution_callback(self):
        """
        Main execution callback that processes the current plan
        """
        if not self.is_executing or not self.current_plan:
            return

        # Acquire lock to safely access plan data
        with self.execution_lock:
            if self.plan_index >= len(self.current_plan):
                # Plan completed
                self.is_executing = False
                self.publish_feedback("Plan completed successfully")
                self.publish_status("PLAN_COMPLETED")
                return

            current_action = self.current_plan[self.plan_index]

        # Execute the current action
        action_name = current_action.get('action', 'unknown')
        parameters = current_action.get('parameters', {})
        description = current_action.get('description', '')

        self.get_logger().info(f"Executing action: {action_name} with params {parameters}")

        # Execute based on action type
        success = self._execute_action(action_name, parameters)

        if success:
            # Move to next action
            with self.execution_lock:
                self.plan_index += 1

            # Publish feedback
            feedback_msg = String()
            feedback_msg.data = f"Completed: {action_name} - {description}"
            self.feedback_publisher.publish(feedback_msg)

            if self.plan_index >= len(self.current_plan):
                # Plan completed
                with self.execution_lock:
                    self.is_executing = False
                self.publish_feedback("Plan completed successfully")
                self.publish_status("PLAN_COMPLETED")
        else:
            # Action failed - stop execution
            with self.execution_lock:
                self.is_executing = False
            self.publish_feedback(f"Action failed: {action_name}")
            self.publish_status("PLAN_FAILED")

    def _execute_action(self, action_name: str, parameters: Dict[str, Any]) -> bool:
        """
        Execute a specific action based on its name and parameters

        Args:
            action_name: Name of the action to execute
            parameters: Parameters for the action

        Returns:
            True if action executed successfully, False otherwise
        """
        try:
            if action_name == 'move_to':
                return self._execute_move_to(parameters)
            elif action_name == 'navigate_to':
                return self._execute_navigate_to(parameters)
            elif action_name == 'grasp':
                return self._execute_grasp(parameters)
            elif action_name == 'release':
                return self._execute_release(parameters)
            elif action_name == 'detect_object':
                return self._execute_detect_object(parameters)
            elif action_name == 'turn':
                return self._execute_turn(parameters)
            elif action_name == 'stop':
                return self._execute_stop(parameters)
            elif action_name == 'wait':
                return self._execute_wait(parameters)
            else:
                self.get_logger().warn(f"Unknown action: {action_name}")
                return False

        except Exception as e:
            self.get_logger().error(f"Error executing action {action_name}: {e}")
            return False

    def _execute_move_to(self, parameters: Dict[str, Any]) -> bool:
        """
        Execute move_to action (simplified - just moves in direction)
        """
        x = parameters.get('x', 0.0)
        y = parameters.get('y', 0.0)
        z = parameters.get('z', 0.0)

        # For demo, just move forward for a fixed distance
        duration = abs(x) / self.linear_speed if abs(x) > 0.1 else 2.0
        cmd = Twist()
        cmd.linear.x = self.linear_speed if x > 0 else -self.linear_speed if x < 0 else 0.0
        cmd.linear.y = self.linear_speed if y > 0 else -self.linear_speed if y < 0 else 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd)
        time.sleep(duration)
        self._stop_robot()

        return True

    def _execute_navigate_to(self, parameters: Dict[str, Any]) -> bool:
        """
        Execute navigate_to action (simplified - just turn toward location)
        """
        location = parameters.get('location', 'unknown')

        # For demo, just turn in place
        cmd = Twist()
        cmd.angular.z = self.angular_speed
        self.cmd_vel_publisher.publish(cmd)
        time.sleep(2.0)  # Turn for 2 seconds
        self._stop_robot()

        self.get_logger().info(f"Navigated toward {location}")
        return True

    def _execute_grasp(self, parameters: Dict[str, Any]) -> bool:
        """
        Execute grasp action (simulated)
        """
        object_name = parameters.get('object_name', 'unknown')

        # In a real robot, this would control grippers or arms
        # For simulation, just log the action
        self.get_logger().info(f"Grasping {object_name}")
        time.sleep(1.0)  # Simulate grasp time

        return True

    def _execute_release(self, parameters: Dict[str, Any]) -> bool:
        """
        Execute release action (simulated)
        """
        object_name = parameters.get('object_name', 'unknown')

        # In a real robot, this would control grippers or arms
        # For simulation, just log the action
        self.get_logger().info(f"Releasing {object_name}")
        time.sleep(1.0)  # Simulate release time

        return True

    def _execute_detect_object(self, parameters: Dict[str, Any]) -> bool:
        """
        Execute detect_object action (simulated)
        """
        object_name = parameters.get('object_name', 'unknown')

        # In a real robot, this would use perception systems
        # For simulation, just return success
        self.get_logger().info(f"Detected {object_name}")
        time.sleep(0.5)  # Simulate detection time

        return True

    def _execute_turn(self, parameters: Dict[str, Any]) -> bool:
        """
        Execute turn action
        """
        direction = parameters.get('direction', 'left')
        angle = parameters.get('angle', 90.0)

        cmd = Twist()
        if direction == 'left':
            cmd.angular.z = self.angular_speed
        else:
            cmd.angular.z = -self.angular_speed

        # Calculate duration based on angle
        duration = abs(angle) * 3.14159 / 180.0 / self.angular_speed
        self.cmd_vel_publisher.publish(cmd)
        time.sleep(duration)
        self._stop_robot()

        return True

    def _execute_stop(self, parameters: Dict[str, Any]) -> bool:
        """
        Execute stop action
        """
        self._stop_robot()
        return True

    def _execute_wait(self, parameters: Dict[str, Any]) -> bool:
        """
        Execute wait action
        """
        duration = parameters.get('duration', 1.0)
        time.sleep(duration)
        return True

    def _stop_robot(self):
        """
        Stop all robot movement
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

    def publish_status(self, status: str):
        """
        Publish status message
        """
        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)

    def publish_feedback(self, feedback: str):
        """
        Publish feedback message
        """
        feedback_msg = String()
        feedback_msg.data = feedback
        self.feedback_publisher.publish(feedback_msg)

    def cancel_execution(self):
        """
        Cancel current plan execution
        """
        with self.execution_lock:
            self.is_executing = False
            self.current_plan = None
            self.plan_index = 0
        self._stop_robot()
        self.publish_status("EXECUTION_CANCELLED")


def main(args=None):
    """
    Main function to run the LLM action sequencer node
    """
    rclpy.init(args=args)

    node = LLMActionSequencer()

    try:
        node.get_logger().info("LLM Action Sequencer node starting...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LLM Action Sequencer node")
        node.cancel_execution()
    finally:
        # Ensure robot stops before shutting down
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()