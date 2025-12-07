#!/usr/bin/env python3
"""
Vision-Based Navigation Agent

This script implements a robot navigation agent that uses both 2D vision (OpenCV) and 3D point cloud data (PCL concepts)
to navigate and avoid obstacles. The agent processes camera images and depth data to detect obstacles and plan safe paths.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs_py import point_cloud2


class VisionNavigationAgent(Node):
    def __init__(self):
        super().__init__('vision_navigation_agent')

        # Create CvBridge instance
        self.bridge = CvBridge()

        # Subscribe to various sensor data
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # Robot state
        self.rgb_image = None
        self.depth_image = None
        self.pointcloud_data = None
        self.latest_scan = None

        # Navigation parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.safe_distance = 1.0  # meters
        self.front_angle_range = 30  # degrees for front detection

        # Vision processing results
        self.obstacle_detected_2d = False
        self.obstacle_distance_3d = float('inf')
        self.navigation_direction = 0  # -1 for left, 0 for forward, 1 for right

        self.get_logger().info('Vision-Based Navigation Agent initialized')

    def rgb_callback(self, msg):
        """Process RGB camera image for 2D obstacle detection"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process image to detect obstacles (simplified approach)
            self.obstacle_detected_2d, direction = self.detect_obstacles_2d(cv_image)
            self.navigation_direction = direction

            # Optional: display processed image
            cv2.imshow('RGB Camera', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {str(e)}')

    def depth_callback(self, msg):
        """Process depth image for 3D obstacle detection"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Process depth to find closest obstacle
            self.obstacle_distance_3d = self.get_closest_obstacle_distance(depth_image)

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def pointcloud_callback(self, msg):
        """Process point cloud data"""
        try:
            # Convert to list of points
            points_list = []
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points_list.append([point[0], point[1], point[2]])

            self.pointcloud_data = np.array(points_list) if points_list else np.array([])

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')

    def detect_obstacles_2d(self, image):
        """
        Detect obstacles in 2D image using color and edge detection
        Returns: (obstacle_detected, preferred_direction)
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply edge detection
        edges = cv2.Canny(gray, 50, 150)

        # Get image dimensions
        height, width = image.shape[:2]

        # Divide image into regions (left, center, right)
        center_region = edges[height//3:2*height//3, width//4:3*width//4]
        left_region = edges[:, :width//3]
        right_region = edges[:, 2*width//3:]

        # Count edges in each region (indicating potential obstacles)
        center_edges = np.sum(center_region > 0)
        left_edges = np.sum(left_region > 0)
        right_edges = np.sum(right_region > 0)

        # Determine if obstacle is detected in front
        obstacle_detected = center_edges > (height * width * 0.01)  # threshold

        # Determine preferred direction based on edge distribution
        if left_edges > right_edges:
            direction = 1  # Prefer right
        elif right_edges > left_edges:
            direction = -1  # Prefer left
        else:
            direction = 0  # Forward is OK

        return obstacle_detected, direction

    def get_closest_obstacle_distance(self, depth_image):
        """
        Find the closest obstacle distance from depth image
        """
        # Create mask for valid depth values (within reasonable range)
        valid_mask = (depth_image > 0.1) & (depth_image < 10.0)

        if np.any(valid_mask):
            # Get minimum depth in the region of interest (center of image)
            height, width = depth_image.shape
            roi = depth_image[height//4:3*height//4, width//4:3*width//4]
            roi_mask = valid_mask[height//4:3*height//4, width//4:3*width//4]

            if np.any(roi_mask):
                return np.min(roi[roi_mask])

        return float('inf')

    def control_loop(self):
        """Main control loop that integrates 2D and 3D vision data"""
        # Combine 2D and 3D obstacle detection results
        obstacle_2d_confirmed = self.obstacle_detected_2d
        obstacle_3d_confirmed = self.obstacle_distance_3d < self.safe_distance

        # Determine navigation command
        cmd = Twist()

        if not obstacle_2d_confirmed and not obstacle_3d_confirmed:
            # No obstacles detected, move forward
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.get_logger().info('Clear path, moving forward')
        else:
            # Obstacles detected, navigate based on vision data
            if self.navigation_direction == -1:  # Prefer left
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed
                self.get_logger().info('Obstacles detected, turning left')
            elif self.navigation_direction == 1:  # Prefer right
                cmd.linear.x = 0.0
                cmd.angular.z = -self.angular_speed
                self.get_logger().info('Obstacles detected, turning right')
            else:  # Turn in a default direction
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed
                self.get_logger().info('Obstacles detected, turning')

        # Publish navigation command
        self.cmd_vel_publisher.publish(cmd)

        # Log current state
        self.get_logger().info(
            f'2D Obstacle: {self.obstacle_detected_2d}, '
            f'3D Distance: {self.obstacle_distance_3d:.2f}m, '
            f'Cmd: lin={cmd.linear.x:.2f}, ang={cmd.angular.z:.2f}'
        )

        # Clean up OpenCV windows periodically
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    agent = VisionNavigationAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Shutting down Vision Navigation Agent')
    finally:
        # Stop the robot before shutting down
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        agent.cmd_vel_publisher.publish(cmd)

        # Clean up OpenCV windows
        cv2.destroyAllWindows()

        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()