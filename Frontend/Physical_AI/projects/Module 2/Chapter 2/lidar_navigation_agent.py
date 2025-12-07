#!/usr/bin/env python3
"""
LIDAR Navigation Agent for Humanoid Robot

This script implements a Python agent that uses LiDAR data to navigate and avoid obstacles
in a simulated environment. The agent subscribes to LiDAR scan data, processes it to detect
obstacles, and publishes velocity commands to navigate safely around obstacles.

The agent uses a simple reactive navigation approach:
- If no obstacles detected in front, move forward
- If obstacles detected in front, turn away from obstacles
- If obstacles detected on both sides, turn in a random direction
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math


class LIDARNavigationAgent(Node):
    def __init__(self):
        super().__init__('lidar_navigation_agent')

        # Subscribe to LiDAR scan data
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/humanoid/laser_scan',
            self.lidar_callback,
            10
        )

        # Subscribe to odometry data (for position tracking)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # Robot state
        self.lidar_data = None
        self.odom_data = None
        self.current_position = (0.0, 0.0)
        self.current_yaw = 0.0

        # Navigation parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.8  # rad/s
        self.safe_distance = 1.0  # meters
        self.front_angle_range = 30  # degrees for front detection

        self.get_logger().info('LIDAR Navigation Agent initialized')

    def lidar_callback(self, msg):
        """Process incoming LiDAR scan data"""
        self.lidar_data = msg
        self.get_logger().debug(f'Received LiDAR data with {len(msg.ranges)} ranges')

    def odom_callback(self, msg):
        """Process incoming odometry data"""
        self.odom_data = msg
        # Extract position
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        # Extract orientation (yaw) from quaternion
        quat = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def get_front_distance(self):
        """Get minimum distance in front of robot"""
        if self.lidar_data is None:
            return float('inf')

        # Calculate the index range for front detection
        angle_min = self.lidar_data.angle_min
        angle_max = self.lidar_data.angle_max
        angle_increment = self.lidar_data.angle_increment

        # Front angle range (e.g., -30 to +30 degrees)
        front_min_angle = math.radians(-self.front_angle_range)
        front_max_angle = math.radians(self.front_angle_range)

        # Convert angles to indices
        start_idx = int((front_min_angle - angle_min) / angle_increment)
        end_idx = int((front_max_angle - angle_min) / angle_increment)

        # Ensure indices are within bounds
        start_idx = max(0, min(start_idx, len(self.lidar_data.ranges) - 1))
        end_idx = max(0, min(end_idx, len(self.lidar_data.ranges) - 1))

        if start_idx >= len(self.lidar_data.ranges) or end_idx < start_idx:
            return float('inf')

        # Get distances in front range and find minimum
        front_distances = self.lidar_data.ranges[start_idx:end_idx + 1]
        front_distances = [d for d in front_distances if not math.isnan(d) and d > 0]

        if not front_distances:
            return float('inf')

        return min(front_distances)

    def get_side_distances(self):
        """Get minimum distances on left and right sides"""
        if self.lidar_data is None:
            return float('inf'), float('inf')

        angle_min = self.lidar_data.angle_min
        angle_increment = self.lidar_data.angle_increment

        # Left side: 30 to 150 degrees
        left_min_angle = math.radians(30)
        left_max_angle = math.radians(150)

        # Right side: -150 to -30 degrees
        right_min_angle = math.radians(-150)
        right_max_angle = math.radians(-30)

        # Convert to indices
        left_start_idx = int((left_min_angle - angle_min) / angle_increment)
        left_end_idx = int((left_max_angle - angle_min) / angle_increment)
        right_start_idx = int((right_min_angle - angle_min) / angle_increment)
        right_end_idx = int((right_max_angle - angle_min) / angle_increment)

        # Ensure indices are within bounds
        left_start_idx = max(0, min(left_start_idx, len(self.lidar_data.ranges) - 1))
        left_end_idx = max(0, min(left_end_idx, len(self.lidar_data.ranges) - 1))
        right_start_idx = max(0, min(right_start_idx, len(self.lidar_data.ranges) - 1))
        right_end_idx = max(0, min(right_end_idx, len(self.lidar_data.ranges) - 1))

        # Get left distances
        left_distances = self.lidar_data.ranges[left_start_idx:left_end_idx + 1]
        left_distances = [d for d in left_distances if not math.isnan(d) and d > 0]
        left_min_dist = min(left_distances) if left_distances else float('inf')

        # Get right distances
        right_distances = self.lidar_data.ranges[right_start_idx:right_end_idx + 1]
        right_distances = [d for d in right_distances if not math.isnan(d) and d > 0]
        right_min_dist = min(right_distances) if right_distances else float('inf')

        return left_min_dist, right_min_dist

    def navigate(self):
        """Main navigation logic based on LiDAR data"""
        if self.lidar_data is None:
            # If no LiDAR data, stop the robot
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd

        # Get distance measurements
        front_dist = self.get_front_distance()
        left_dist, right_dist = self.get_side_distances()

        cmd = Twist()

        # Navigation logic
        if front_dist > self.safe_distance:
            # No obstacles in front, move forward
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.get_logger().debug(f'Clear path ahead, moving forward. Front: {front_dist:.2f}')
        elif left_dist > right_dist:
            # Turn left (obstacle closer on right)
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            self.get_logger().debug(f'Turning left. Left: {left_dist:.2f}, Right: {right_dist:.2f}')
        elif right_dist > left_dist:
            # Turn right (obstacle closer on left)
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed
            self.get_logger().debug(f'Turning right. Left: {left_dist:.2f}, Right: {right_dist:.2f}')
        else:
            # Equal distances or no clear path, turn randomly
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed  # Default to turning left
            self.get_logger().debug(f'Equal distances, turning left. Left: {left_dist:.2f}, Right: {right_dist:.2f}')

        # Apply safety limits
        cmd.linear.x = max(0.0, min(cmd.linear.x, self.linear_speed))
        cmd.angular.z = max(-self.angular_speed, min(cmd.angular.z, self.angular_speed))

        return cmd

    def control_loop(self):
        """Main control loop"""
        if self.lidar_data is not None:
            # Get navigation command
            cmd = self.navigate()

            # Publish command
            self.cmd_vel_publisher.publish(cmd)

            # Log current state
            front_dist = self.get_front_distance()
            self.get_logger().info(f'Front distance: {front_dist:.2f}m, Cmd: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}')
        else:
            # If no data, stop the robot
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd)
            self.get_logger().warn('No LiDAR data available, stopping robot')


def main(args=None):
    rclpy.init(args=args)

    agent = LIDARNavigationAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Shutting down LIDAR Navigation Agent')
    finally:
        # Stop the robot before shutting down
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        agent.cmd_vel_publisher.publish(cmd)
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()