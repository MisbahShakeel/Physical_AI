#!/usr/bin/env python3
# simulated_joint_control.py - Simulated humanoid joint control using ROS 2 Control with basic PID

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import math
from collections import deque


class SimplePIDController:
    """
    A simple PID controller implementation for joint control.
    This simulates the functionality of ROS 2 Control's PID controllers.
    """

    def __init__(self, kp=100.0, ki=0.01, kd=10.0, dt=0.01):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.dt = dt  # Time step

        self.previous_error = 0.0
        self.integral_error = 0.0

    def compute(self, setpoint, measured_value):
        """
        Compute the control output based on setpoint and measured value.

        Args:
            setpoint: Desired value
            measured_value: Current measured value

        Returns:
            Control output (effort/command)
        """
        error = setpoint - measured_value

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral_error += error * self.dt
        i_term = self.ki * self.integral_error

        # Derivative term
        derivative = (error - self.previous_error) / self.dt
        d_term = self.kd * derivative

        # Store current error for next derivative calculation
        self.previous_error = error

        # Total control output
        output = p_term + i_term + d_term

        return output


class SimulatedJointControl(Node):
    """
    Simulates humanoid joint control using a basic PID controller.
    This node demonstrates the principles behind ROS 2 Control without requiring
    actual hardware or the full ROS 2 Control framework.
    """

    def __init__(self):
        super().__init__('simulated_joint_control')

        # Publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/simulated_joint_commands',
            10
        )

        # Publisher for joint states (simulated feedback)
        self.joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Subscriber for trajectory commands
        self.trajectory_subscriber = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.trajectory_callback,
            10
        )

        # Timer for the control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

        # Initialize joint state
        self.joint_names = ['left_elbow_joint', 'right_elbow_joint', 'left_knee_joint']
        self.current_positions = [0.0, 0.0, 0.0]  # Current joint positions (radians)
        self.current_velocities = [0.0, 0.0, 0.0]  # Current joint velocities (rad/s)
        self.current_efforts = [0.0, 0.0, 0.0]    # Current joint efforts (N*m)

        # Desired positions (setpoints)
        self.desired_positions = [0.0, 0.0, 0.0]

        # PID controllers for each joint
        self.pid_controllers = [
            SimplePIDController(kp=50.0, ki=0.1, kd=5.0),  # Left elbow
            SimplePIDController(kp=50.0, ki=0.1, kd=5.0),  # Right elbow
            SimplePIDController(kp=80.0, ki=0.2, kd=8.0)   # Left knee (higher stiffness)
        ]

        # Trajectory queue for smooth motion execution
        self.trajectory_queue = deque()
        self.current_trajectory_point = None
        self.trajectory_start_time = None

        # Simulation parameters
        self.simulation_time = 0.0
        self.dt = 0.01  # 10ms time step

        self.get_logger().info('Simulated Joint Control node initialized')

    def trajectory_callback(self, msg):
        """Callback to handle incoming trajectory commands."""
        if len(msg.points) > 0:
            # Add trajectory points to the queue
            self.trajectory_queue.clear()  # Clear existing trajectory
            for point in msg.points:
                self.trajectory_queue.append(point)

            # Set the start time for trajectory execution
            self.trajectory_start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f'Received trajectory with {len(msg.points)} points')

    def control_loop(self):
        """Main control loop that runs at 100Hz."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.simulation_time += self.dt

        # Process trajectory if available
        if self.trajectory_queue and self.trajectory_start_time is not None:
            self.execute_trajectory()

        # Update joint dynamics using simple physics simulation
        self.update_joint_dynamics()

        # Publish joint states for visualization/feedback
        self.publish_joint_states()

    def execute_trajectory(self):
        """Execute the current trajectory point."""
        if not self.trajectory_queue:
            return

        # Get the next trajectory point if available
        if (self.current_trajectory_point is None or
            (self.get_clock().now().nanoseconds / 1e9 - self.trajectory_start_time) >=
            self.current_trajectory_point.time_from_start.sec +
            self.current_trajectory_point.time_from_start.nanosec / 1e9):

            if self.trajectory_queue:
                self.current_trajectory_point = self.trajectory_queue.popleft()

                # Update desired positions based on the trajectory point
                for i, joint_name in enumerate(self.joint_names):
                    if joint_name in self.current_trajectory_point.positions:
                        idx = self.joint_names.index(joint_name)
                        if idx < len(self.current_trajectory_point.positions):
                            self.desired_positions[idx] = self.current_trajectory_point.positions[idx]
                    else:
                        # If joint not in trajectory, keep current desired position
                        pass

                self.get_logger().info(f'Executing trajectory point: {self.desired_positions}')

    def update_joint_dynamics(self):
        """
        Update joint positions, velocities, and efforts based on PID control.
        This simulates the physical behavior of the joints.
        """
        for i in range(len(self.joint_names)):
            # Get control effort from PID controller
            control_effort = self.pid_controllers[i].compute(
                self.desired_positions[i],
                self.current_positions[i]
            )

            # Apply simple dynamics simulation (F = ma, but for rotational joints)
            # Torque = Inertia * AngularAcceleration
            # For simplicity, assume unit inertia and add some damping
            angular_acceleration = control_effort - 0.1 * self.current_velocities[i]  # Damping

            # Update velocities and positions using Euler integration
            self.current_velocities[i] += angular_acceleration * self.dt
            self.current_positions[i] += self.current_velocities[i] * self.dt

            # Update efforts (for completeness)
            self.current_efforts[i] = control_effort

    def publish_joint_states(self):
        """Publish current joint states for visualization and other nodes."""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.current_positions
        joint_state_msg.velocity = self.current_velocities
        joint_state_msg.effort = self.current_efforts

        self.joint_state_publisher.publish(joint_state_msg)

    def set_joint_positions(self, joint_positions):
        """
        Set desired joint positions directly (bypassing trajectories).

        Args:
            joint_positions: List of desired joint positions
        """
        if len(joint_positions) == len(self.joint_names):
            self.desired_positions = list(joint_positions)
            self.get_logger().info(f'Set desired positions: {self.desired_positions}')
        else:
            self.get_logger().error(f'Number of positions ({len(joint_positions)}) does not match number of joints ({len(self.joint_names)})')

    def add_trajectory_point(self, positions, duration=2.0):
        """
        Add a trajectory point to be executed.

        Args:
            positions: List of joint positions for this point
            duration: Time to reach this point from the previous one
        """
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)  # Zero velocity at goal
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration - int(duration)) * 1e9)
        )

        self.trajectory_queue.append(point)
        self.get_logger().info(f'Added trajectory point: {positions}, duration: {duration}s')

    def perform_dance_sequence(self):
        """Perform a simple dance sequence to demonstrate control."""
        # Clear any existing trajectory
        self.trajectory_queue.clear()

        # Define a sequence of poses for a simple dance
        dance_poses = [
            [0.0, 0.0, 0.0],      # Home position
            [0.5, -0.5, 0.2],     # Pose 1
            [-0.3, 0.3, -0.2],    # Pose 2
            [0.0, 0.0, 0.0],      # Return home
            [0.7, 0.2, 0.5],      # Pose 3
            [-0.4, -0.4, -0.3],   # Pose 4
            [0.0, 0.0, 0.0]       # Return home
        ]

        # Add each pose to the trajectory with 2 seconds duration
        for i, pose in enumerate(dance_poses):
            self.add_trajectory_point(pose, 2.0)
            self.get_logger().info(f'Added dance pose {i+1}: {pose}')


def main(args=None):
    """Main function to run the simulated joint control node."""
    rclpy.init(args=args)

    controller = SimulatedJointControl()

    try:
        controller.get_logger().info('Starting Simulated Joint Control...')

        # Optionally start a dance sequence after initialization
        # controller.perform_dance_sequence()

        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down Simulated Joint Control node')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()