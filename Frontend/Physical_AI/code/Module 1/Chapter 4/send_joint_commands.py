#!/usr/bin/env python3
# send_joint_commands.py - Python script to send commands to a ROS 2 Control joint controller

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import math


class JointCommandSender(Node):
    """
    A node that sends commands to ROS 2 Control joint controllers.
    Demonstrates different ways to command joints: direct commands and trajectories.
    """

    def __init__(self):
        super().__init__('joint_command_sender')

        # Publisher for direct joint commands (position, velocity, effort)
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',  # Topic for position commands
            10
        )

        # Publisher for trajectory commands
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Action client for trajectory execution (more sophisticated control)
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory'
        )

        # Timer to send commands periodically
        self.command_timer = self.create_timer(3.0, self.send_periodic_commands)

        # Command sequence counter
        self.command_sequence = 0

        self.get_logger().info('Joint Command Sender node initialized')

    def send_position_command(self, joint_names, positions):
        """
        Send direct position commands to the controller.

        Args:
            joint_names: List of joint names to command
            positions: List of target positions for each joint
        """
        # For direct position controllers, we use Float64MultiArray
        command_msg = Float64MultiArray()
        command_msg.data = positions

        self.joint_command_publisher.publish(command_msg)
        self.get_logger().info(f'Sent position command: {dict(zip(joint_names, positions))}')

    def send_trajectory_command(self, joint_names, positions, velocities=None, accelerations=None, duration=2.0):
        """
        Send a trajectory command to the controller.

        Args:
            joint_names: List of joint names to control
            positions: List of target positions for each joint
            velocities: List of target velocities for each joint (optional)
            accelerations: List of target accelerations for each joint (optional)
            duration: Duration for the trajectory execution in seconds
        """
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions

        if velocities is not None:
            point.velocities = velocities
        else:
            # Default to zero velocities if not provided
            point.velocities = [0.0] * len(positions)

        if accelerations is not None:
            point.accelerations = accelerations
        else:
            # Default to zero accelerations if not provided
            point.accelerations = [0.0] * len(positions)

        # Convert duration to ROS time
        point.time_from_start = Duration()
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        trajectory_msg.points.append(point)

        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info(f'Sent trajectory command: positions={positions}, duration={duration}s')

    def send_trajectory_goal(self, joint_names, trajectory_points):
        """
        Send a trajectory goal using the action interface for more sophisticated control.

        Args:
            joint_names: List of joint names to control
            trajectory_points: List of trajectory points with positions, velocities, etc.
        """
        # Wait for the action server to be available
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return

        # Create the goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = trajectory_points

        # Send the goal
        self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.trajectory_feedback_callback
        )

        self.get_logger().info(f'Sent trajectory goal with {len(trajectory_points)} points')

    def trajectory_feedback_callback(self, feedback_msg):
        """Callback function to handle trajectory execution feedback."""
        self.get_logger().debug('Received trajectory feedback')

    def send_periodic_commands(self):
        """
        Send a sequence of different types of commands periodically.
        This demonstrates various ways to command a joint controller.
        """
        joint_names = ['left_elbow_joint', 'right_elbow_joint']  # Example joint names

        if self.command_sequence == 0:
            # Send a simple position command to move to a specific angle
            positions = [0.5, -0.3]  # Radians
            self.send_position_command(joint_names, positions)
            self.get_logger().info('Command sequence 0: Sent position command')
        elif self.command_sequence == 1:
            # Send a trajectory command with smooth motion
            positions = [0.0, 0.0]  # Return to home position
            velocities = [0.1, 0.1]  # Smooth velocities
            self.send_trajectory_command(joint_names, positions, velocities, duration=3.0)
            self.get_logger().info('Command sequence 1: Sent trajectory command')
        elif self.command_sequence == 2:
            # Send a sinusoidal motion pattern
            t = self.get_clock().now().nanoseconds / 1e9  # Time in seconds
            pos1 = 0.5 * math.sin(t)  # Sinusoidal motion for joint 1
            pos2 = 0.5 * math.cos(t)  # Cosine motion for joint 2
            positions = [pos1, pos2]
            self.send_position_command(joint_names, positions)
            self.get_logger().info(f'Command sequence 2: Sent sinusoidal command at t={t:.2f}')
        elif self.command_sequence == 3:
            # Send a complex trajectory with multiple points
            trajectory_points = []

            # Point 1: Start position
            point1 = JointTrajectoryPoint()
            point1.positions = [0.0, 0.0]
            point1.velocities = [0.0, 0.0]
            point1.time_from_start = Duration(sec=0, nanosec=0)
            trajectory_points.append(point1)

            # Point 2: Mid position
            point2 = JointTrajectoryPoint()
            point2.positions = [0.5, -0.5]
            point2.velocities = [0.1, -0.1]
            point2.time_from_start = Duration(sec=2, nanosec=0)
            trajectory_points.append(point2)

            # Point 3: End position
            point3 = JointTrajectoryPoint()
            point3.positions = [0.0, 0.0]
            point3.velocities = [0.0, 0.0]
            point3.time_from_start = Duration(sec=4, nanosec=0)
            trajectory_points.append(point3)

            self.send_trajectory_goal(joint_names, trajectory_points)
            self.get_logger().info('Command sequence 3: Sent complex trajectory goal')

        # Increment and wrap around the sequence
        self.command_sequence = (self.command_sequence + 1) % 4

    def send_home_position(self):
        """Send command to move all joints to home position (0.0)."""
        joint_names = ['left_elbow_joint', 'right_elbow_joint']
        home_positions = [0.0, 0.0]
        self.send_position_command(joint_names, home_positions)
        self.get_logger().info('Sent home position command')

    def send_sine_wave_pattern(self, duration=10.0, frequency=0.5):
        """
        Send a continuous sine wave pattern for demonstration.

        Args:
            duration: Duration of the pattern in seconds
            frequency: Frequency of the sine wave in Hz
        """
        start_time = self.get_clock().now().nanoseconds / 1e9

        while (self.get_clock().now().nanoseconds / 1e9 - start_time) < duration:
            current_time = self.get_clock().now().nanoseconds / 1e9
            t = current_time - start_time

            # Generate sine wave positions
            pos1 = 0.5 * math.sin(2 * math.pi * frequency * t)
            pos2 = 0.5 * math.cos(2 * math.pi * frequency * t)

            joint_names = ['left_elbow_joint', 'right_elbow_joint']
            positions = [pos1, pos2]

            self.send_position_command(joint_names, positions)

            # Sleep briefly to control the update rate
            time.sleep(0.1)


def main(args=None):
    """Main function to run the joint command sender node."""
    rclpy.init(args=args)

    command_sender = JointCommandSender()

    try:
        command_sender.get_logger().info('Starting Joint Command Sender...')
        rclpy.spin(command_sender)
    except KeyboardInterrupt:
        command_sender.get_logger().info('Shutting down Joint Command Sender node')
    finally:
        command_sender.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()