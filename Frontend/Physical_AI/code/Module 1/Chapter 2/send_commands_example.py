#!/usr/bin/env python3
# send_commands_example.py - Example of sending commands to a ROS 2 controller from rclpy

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointJog
import time


class CommandSender(Node):
    """
    Example node that demonstrates sending commands to a ROS 2 controller.
    This simulates sending commands to a mock controller for demonstration purposes.
    """

    def __init__(self):
        super().__init__('command_sender')

        # Publisher for sending position commands to the controller
        self.position_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )

        # Publisher for sending velocity commands to the controller
        self.velocity_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10
        )

        # Publisher for sending trajectory commands to the controller
        self.trajectory_command_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Publisher for sending jog commands to the controller
        self.jog_command_publisher = self.create_publisher(
            JointJog,
            '/joint_jog_controller/joint_jog',
            10
        )

        # Subscriber for receiving joint states (feedback from the controller)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Store current joint states for feedback
        self.current_joint_states = JointState()

        # Timer for sending commands periodically
        self.command_timer = self.create_timer(2.0, self.send_periodic_commands)

        # Command sequence counter
        self.command_sequence = 0

        self.get_logger().info('Command Sender node initialized')

    def joint_state_callback(self, msg):
        """Callback to receive joint state feedback from the controller."""
        self.current_joint_states = msg
        self.get_logger().debug(f'Received joint feedback: {len(msg.name)} joints')

    def send_position_command(self, joint_positions):
        """
        Send position commands to the controller.

        Args:
            joint_positions: List of target positions for each joint
        """
        command_msg = Float64MultiArray()
        command_msg.data = joint_positions
        self.position_command_publisher.publish(command_msg)
        self.get_logger().info(f'Sent position command: {joint_positions}')

    def send_velocity_command(self, joint_velocities):
        """
        Send velocity commands to the controller.

        Args:
            joint_velocities: List of target velocities for each joint
        """
        command_msg = Float64MultiArray()
        command_msg.data = joint_velocities
        self.velocity_command_publisher.publish(command_msg)
        self.get_logger().info(f'Sent velocity command: {joint_velocities}')

    def send_trajectory_command(self, joint_names, positions, velocities=None, duration=3.0):
        """
        Send a trajectory command to the controller.

        Args:
            joint_names: List of joint names to control
            positions: List of target positions for each joint
            velocities: List of target velocities for each joint (optional)
            duration: Duration for the trajectory execution in seconds
        """
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions

        if velocities is not None:
            point.velocities = velocities

        # Convert duration to ROS time
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        trajectory_msg.points.append(point)

        self.trajectory_command_publisher.publish(trajectory_msg)
        self.get_logger().info(f'Sent trajectory command: positions={positions}, duration={duration}s')

    def send_jog_command(self, joint_names, displacements, velocities):
        """
        Send jog commands for incremental motion.

        Args:
            joint_names: List of joint names to jog
            displacements: List of displacement values for each joint
            velocities: List of velocity values for each joint
        """
        jog_msg = JointJog()
        jog_msg.joint_names = joint_names
        jog_msg.displacements = displacements
        jog_msg.velocities = velocities
        jog_msg.duration = 0.1  # Jog for 0.1 seconds

        self.jog_command_publisher.publish(jog_msg)
        self.get_logger().info(f'Sent jog command: joints={joint_names}, displacements={displacements}')

    def send_periodic_commands(self):
        """
        Send a sequence of different types of commands periodically.
        This demonstrates various ways to command a controller.
        """
        # Define joint names for our mock robot
        joint_names = ['joint1', 'joint2', 'joint3']

        if self.command_sequence == 0:
            # Send position command
            positions = [0.5, -0.3, 1.2]
            self.send_position_command(positions)
        elif self.command_sequence == 1:
            # Send velocity command
            velocities = [0.1, -0.05, 0.2]
            self.send_velocity_command(velocities)
        elif self.command_sequence == 2:
            # Send trajectory command
            positions = [0.0, 0.0, 0.0]  # Return to home position
            velocities = [0.0, 0.0, 0.0]
            self.send_trajectory_command(joint_names, positions, velocities, 2.0)
        elif self.command_sequence == 3:
            # Send jog command
            displacements = [0.1, 0.05, -0.05]
            velocities = [0.2, 0.1, -0.1]
            self.send_jog_command(joint_names, displacements, velocities)

        # Increment and wrap around the sequence
        self.command_sequence = (self.command_sequence + 1) % 4

    def send_home_position(self):
        """Send command to move all joints to home position (0.0)."""
        num_joints = 3  # Assuming a 3-DOF robot
        home_positions = [0.0] * num_joints
        self.send_position_command(home_positions)
        self.get_logger().info('Sent home position command')

    def send_test_trajectory(self):
        """Send a test trajectory to demonstrate path following."""
        joint_names = ['joint1', 'joint2', 'joint3']

        # Define a sequence of positions to move through
        positions_sequence = [
            [0.0, 0.0, 0.0],      # Home
            [0.5, 0.3, -0.2],     # Position 1
            [0.8, -0.2, 0.5],     # Position 2
            [0.0, 0.0, 0.0]       # Return home
        ]

        for i, positions in enumerate(positions_sequence):
            self.get_logger().info(f'Sending trajectory point {i+1}: {positions}')
            self.send_trajectory_command(joint_names, positions, duration=2.0)
            time.sleep(2.1)  # Wait for movement to complete


def main(args=None):
    """Main function to run the command sender node."""
    rclpy.init(args=args)

    command_sender = CommandSender()

    try:
        # Optionally send a test trajectory at startup
        command_sender.get_logger().info('Starting command sender...')

        # Run the node
        rclpy.spin(command_sender)
    except KeyboardInterrupt:
        command_sender.get_logger().info('Shutting down Command Sender node')
    finally:
        command_sender.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()