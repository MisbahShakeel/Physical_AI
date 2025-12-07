#!/usr/bin/env python3
# basic_controller_node.py - Basic ROS 2 controller interface node example

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64MultiArray


class BasicControllerInterface(Node):
    """
    Basic controller interface node that demonstrates how to interface
    with ROS 2 controllers using various communication patterns.
    """

    def __init__(self):
        super().__init__('basic_controller_interface')

        # Publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Publisher for individual joint commands (alternative approach)
        self.joint_group_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )

        # Subscriber for joint states (feedback from controllers)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Action server for more complex control tasks
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            self.execute_trajectory_callback
        )

        # Store current joint states
        self.current_joint_states = JointState()

        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info('Basic Controller Interface node initialized')

    def joint_state_callback(self, msg):
        """Callback function to receive joint state updates from controllers."""
        self.current_joint_states = msg
        self.get_logger().debug(f'Received joint states: {len(msg.name)} joints')

    def send_joint_command(self, joint_names, positions, duration=5.0):
        """
        Send a joint trajectory command to the controller.

        Args:
            joint_names: List of joint names to control
            positions: List of target positions for each joint
            duration: Duration for the trajectory execution in seconds
        """
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions

        # Convert duration to ROS time
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        trajectory_msg.points.append(point)

        # Publish the trajectory command
        self.joint_command_publisher.publish(trajectory_msg)
        self.get_logger().info(f'Sent joint command: {dict(zip(joint_names, positions))}')

    def send_position_commands(self, commands):
        """
        Send individual position commands to a controller.

        Args:
            commands: List of position commands
        """
        command_msg = Float64MultiArray()
        command_msg.data = commands
        self.joint_group_publisher.publish(command_msg)
        self.get_logger().info(f'Sent position commands: {commands}')

    def execute_trajectory_callback(self, goal_handle):
        """
        Callback function for the trajectory execution action.

        Args:
            goal_handle: The action goal handle
        """
        self.get_logger().info('Received trajectory execution goal')

        # In a real implementation, you would execute the trajectory here
        # For this example, we'll just accept the goal and report success

        # Check if the goal is valid
        goal = goal_handle.request.goal
        if len(goal.trajectory.points) == 0:
            self.get_logger().error('Invalid trajectory: no points provided')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            return result

        # Simulate trajectory execution
        feedback_msg = FollowJointTrajectory.Feedback()

        for i, point in enumerate(goal.trajectory.points):
            # Publish the trajectory point
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = goal.trajectory.joint_names
            trajectory_msg.points = [point]
            self.joint_command_publisher.publish(trajectory_msg)

            # Send feedback
            feedback_msg.joint_names = goal.trajectory.joint_names
            feedback_msg.actual.positions = point.positions
            feedback_msg.desired.positions = point.positions
            goal_handle.publish_feedback(feedback_msg)

            # Sleep to simulate execution time (in a real implementation,
            # you would wait for actual controller feedback)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        # Complete the goal
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('Trajectory execution completed successfully')

        return result

    def status_callback(self):
        """Periodic callback to report controller status."""
        if self.current_joint_states.name:
            self.get_logger().info(
                f'Controller status: {len(self.current_joint_states.name)} joints active'
            )


def main(args=None):
    """Main function to run the basic controller interface node."""
    rclpy.init(args=args)

    controller_interface = BasicControllerInterface()

    try:
        rclpy.spin(controller_interface)
    except KeyboardInterrupt:
        controller_interface.get_logger().info('Shutting down Basic Controller Interface node')
    finally:
        controller_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()