#!/usr/bin/env python3
# joint_commander_agent.py - Python agent that publishes joint commands to a simulated robot

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time
from collections import deque


class JointCommanderAgent(Node):
    """
    A Python agent that implements higher-level logic to control a simulated robot.
    This agent processes sensor data, makes decisions, and sends appropriate commands
    to the robot's controllers based on the desired behavior.
    """

    def __init__(self):
        super().__init__('joint_commander_agent')

        # Publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )

        # Publisher for trajectory commands (alternative control method)
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscriber for joint states (feedback from the robot)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Store joint state history for analysis
        self.joint_state_history = deque(maxlen=100)
        self.current_joint_states = JointState()

        # Timer for the agent's decision-making loop
        self.agent_timer = self.create_timer(0.1, self.agent_decision_loop)  # 10 Hz

        # Agent state variables
        self.agent_state = 'INIT'  # INIT, IDLE, MOVING, FOLLOWING, etc.
        self.target_positions = [0.0, 0.0, 0.0]  # Default home position
        self.motion_sequence = 0
        self.last_command_time = self.get_clock().now()

        # Initialize agent behavior parameters
        self.initialize_agent_behavior()

        self.get_logger().info('Joint Commander Agent initialized')

    def initialize_agent_behavior(self):
        """Initialize the agent's behavior parameters and state."""
        self.get_logger().info('Initializing agent behavior parameters')
        # Set initial target to home position
        self.set_target_position([0.0, 0.0, 0.0])

    def joint_state_callback(self, msg):
        """Callback function to receive joint state feedback from the robot."""
        self.current_joint_states = msg

        # Add to history for analysis
        self.joint_state_history.append({
            'timestamp': self.get_clock().now(),
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'effort': list(msg.effort)
        })

        # Log joint positions periodically
        if len(self.joint_state_history) % 10 == 0:  # Every 10 updates
            self.get_logger().debug(f'Joint positions: {msg.position}')

    def set_target_position(self, positions):
        """
        Set the target joint positions for the robot.

        Args:
            positions: List of target positions for each joint
        """
        self.target_positions = positions
        self.get_logger().info(f'Set target positions: {positions}')

    def send_joint_command(self, positions):
        """
        Send joint position commands to the robot controller.

        Args:
            positions: List of target positions for each joint
        """
        command_msg = Float64MultiArray()
        command_msg.data = positions
        self.joint_command_publisher.publish(command_msg)

        # Update agent state
        self.agent_state = 'MOVING'
        self.last_command_time = self.get_clock().now()

        self.get_logger().info(f'Sent joint command: {positions}')

    def send_trajectory_command(self, joint_names, positions, duration=2.0):
        """
        Send a trajectory command to move to specific positions.

        Args:
            joint_names: List of joint names
            positions: List of target positions
            duration: Time to reach the target in seconds
        """
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions

        # Convert duration to ROS time
        point.time_from_start = Duration()
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        trajectory_msg.points.append(point)

        self.trajectory_publisher.publish(trajectory_msg)

        # Update agent state
        self.agent_state = 'MOVING'
        self.last_command_time = self.get_clock().now()

        self.get_logger().info(f'Sent trajectory command: positions={positions}, duration={duration}s')

    def calculate_inverse_kinematics(self, target_pose):
        """
        Simple inverse kinematics calculation (for demonstration purposes).
        In a real system, this would use a more sophisticated IK solver.

        Args:
            target_pose: Target end-effector pose (x, y, z, roll, pitch, yaw)

        Returns:
            Joint positions that achieve the target pose (simplified)
        """
        # Simplified example for a 3-DOF planar arm
        x, y, z = target_pose[0], target_pose[1], target_pose[2]

        # Calculate joint angles for a simple 2D planar arm (plus 1 DOF for z)
        # This is a simplified example - real IK would be more complex
        link1_length = 0.5  # Length of first link
        link2_length = 0.4  # Length of second link

        # Calculate distance from origin to target
        dist = math.sqrt(x*x + y*y)

        # Simple inverse kinematics for 2-DOF planar arm
        # This is a simplified approach for demonstration
        if dist <= (link1_length + link2_length):
            # Target is reachable
            # Calculate joint angles using law of cosines
            cos_angle2 = (link1_length**2 + link2_length**2 - dist**2) / (2 * link1_length * link2_length)
            angle2 = math.acos(max(-1, min(1, cos_angle2)))  # Clamp to valid range
            k1 = link1_length + link2_length * math.cos(angle2)
            k2 = link2_length * math.sin(angle2)
            angle1 = math.atan2(y, x) - math.atan2(k2, k1)

            # Calculate third joint for z-axis (simple mapping)
            angle3 = z * 0.5  # Scale z to joint3 movement

            return [angle1, angle2, angle3]
        else:
            # Target is not reachable, return current position
            if self.current_joint_states.position:
                return list(self.current_joint_states.position)
            else:
                return [0.0, 0.0, 0.0]

    def agent_decision_loop(self):
        """
        Main decision-making loop for the agent.
        Based on the current state and sensor feedback, decide on actions.
        """
        current_time = self.get_clock().now()

        if self.agent_state == 'INIT':
            # Initialize - go to home position
            self.send_joint_command([0.0, 0.0, 0.0])
            self.agent_state = 'IDLE'
            self.get_logger().info('Agent initialized, moving to home position')
        elif self.agent_state == 'IDLE':
            # Wait for some time, then perform a predefined motion sequence
            if (current_time - self.last_command_time).nanoseconds > 3e9:  # 3 seconds
                self.execute_motion_sequence()
        elif self.agent_state == 'MOVING':
            # Check if movement has completed (simplified check)
            # In a real system, you'd check if actual position is close to target
            if (current_time - self.last_command_time).nanoseconds > 2.5e9:  # 2.5 seconds
                self.agent_state = 'IDLE'
                self.last_command_time = current_time
                self.get_logger().info('Movement completed')

    def execute_motion_sequence(self):
        """Execute a predefined motion sequence."""
        # Different motions based on sequence number
        if self.motion_sequence == 0:
            # Go to a specific position
            target = [0.5, 0.3, 0.2]
            self.send_joint_command(target)
            self.get_logger().info('Executing motion sequence 0: Moving to position')
        elif self.motion_sequence == 1:
            # Go to another position
            target = [-0.3, 0.5, -0.1]
            self.send_joint_command(target)
            self.get_logger().info('Executing motion sequence 1: Moving to position')
        elif self.motion_sequence == 2:
            # Perform a circular motion (calculate via IK)
            # This is a simplified circular motion in the XY plane
            angle = (self.get_clock().now().nanoseconds / 1e9) % (2 * math.pi)
            x = 0.4 * math.cos(angle)
            y = 0.4 * math.sin(angle)
            z = 0.0  # Keep at same height

            # Calculate joint positions for this Cartesian position
            ik_solution = self.calculate_inverse_kinematics([x, y, z])
            self.send_joint_command(ik_solution)
            self.get_logger().info(f'Executing motion sequence 2: Circular motion at angle={angle:.2f}')
        elif self.motion_sequence == 3:
            # Return to home position
            self.send_joint_command([0.0, 0.0, 0.0])
            self.get_logger().info('Executing motion sequence 3: Returning home')

        # Cycle through sequences
        self.motion_sequence = (self.motion_sequence + 1) % 4

    def compute_smooth_trajectory(self, start_positions, end_positions, steps=20):
        """
        Compute a smooth trajectory between start and end positions.

        Args:
            start_positions: Starting joint positions
            end_positions: Target joint positions
            steps: Number of intermediate steps

        Returns:
            List of intermediate positions
        """
        trajectory = []

        for step in range(steps + 1):
            ratio = step / steps
            intermediate_positions = [
                start + ratio * (end - start)
                for start, end in zip(start_positions, end_positions)
            ]
            trajectory.append(intermediate_positions)

        return trajectory

    def send_smooth_trajectory(self, end_positions, duration=3.0):
        """
        Send a smooth trajectory to move from current position to end position.

        Args:
            end_positions: Target joint positions
            duration: Total time for the movement in seconds
        """
        # Get current position as start
        if self.current_joint_states.position:
            start_positions = list(self.current_joint_states.position)
        else:
            start_positions = [0.0, 0.0, 0.0]

        # Compute smooth trajectory
        trajectory = self.compute_smooth_trajectory(start_positions, end_positions)

        # Send each point in the trajectory
        step_duration = duration / len(trajectory) if len(trajectory) > 0 else 0.1

        for i, positions in enumerate(trajectory):
            # In a real implementation, you might want to send the whole trajectory
            # as a single message, but for this example we'll send point by point
            if i == len(trajectory) - 1:  # Last point - send as trajectory message
                joint_names = ['joint1', 'joint2', 'joint3']  # Assuming 3 joints
                self.send_trajectory_command(joint_names, positions, step_duration)
            else:
                self.send_joint_command(positions)
                time.sleep(step_duration * 0.1)  # Brief pause between points

    def get_current_joint_positions(self):
        """Get the current joint positions from feedback."""
        if self.current_joint_states.position:
            return list(self.current_joint_states.position)
        else:
            return [0.0, 0.0, 0.0]


def main(args=None):
    """Main function to run the joint commander agent."""
    rclpy.init(args=args)

    agent = JointCommanderAgent()

    try:
        agent.get_logger().info('Starting Joint Commander Agent...')
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Shutting down Joint Commander Agent')
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()