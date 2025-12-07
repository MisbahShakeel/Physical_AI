#!/usr/bin/env python3
"""
Depth Perception and 3D Reconstruction Demo

This script demonstrates how to process depth camera data to extract 3D information
and perform basic depth perception tasks using OpenCV and numpy.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthPerceptionDemo(Node):
    def __init__(self):
        super().__init__('depth_perception_demo')

        # Create CvBridge instance
        self.bridge = CvBridge()

        # Subscribe to depth image and camera info
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',  # Replace with your depth topic
            self.depth_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',  # Replace with your camera info topic
            self.camera_info_callback,
            10
        )

        # Store camera intrinsics
        self.camera_matrix = None
        self.distortion_coeffs = None

        # Flag to indicate if camera info has been received
        self.camera_info_received = False

        self.get_logger().info('Depth Perception Demo node started')

    def camera_info_callback(self, msg):
        """Process camera info to extract intrinsics"""
        if not self.camera_info_received:
            # Extract camera matrix
            self.camera_matrix = np.array(msg.k).reshape(3, 3)

            # Extract distortion coefficients
            self.distortion_coeffs = np.array(msg.d)

            self.camera_info_received = True
            self.get_logger().info('Camera info received and intrinsics extracted')

    def depth_callback(self, msg):
        """Process incoming depth image"""
        if not self.camera_info_received:
            self.get_logger().warn('Waiting for camera info...')
            return

        try:
            # Convert ROS Image message to OpenCV image (depth data)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Process the depth data
            processed_depth = self.process_depth_image(depth_image)

            # Display depth image (normalize for visualization)
            depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            cv2.imshow('Depth Image', depth_normalized)
            cv2.waitKey(1)

            self.get_logger().debug(f'Depth image processed: shape={depth_image.shape}, min={np.min(depth_image):.2f}, max={np.max(depth_image):.2f}')

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def process_depth_image(self, depth_image):
        """
        Process depth image to extract 3D information
        """
        # Create a mask for valid depth values (non-zero and within reasonable range)
        valid_mask = (depth_image > 0.1) & (depth_image < 10.0)

        # Calculate some statistics
        if np.any(valid_mask):
            mean_depth = np.mean(depth_image[valid_mask])
            std_depth = np.std(depth_image[valid_mask])

            self.get_logger().info(f'Depth statistics - Mean: {mean_depth:.2f}m, Std: {std_depth:.2f}m')

        # Convert depth image to 3D point cloud
        point_cloud = self.depth_to_point_cloud(depth_image)

        # Find planar surfaces (e.g., ground plane) using RANSAC
        if point_cloud.size > 0:
            plane_params = self.find_plane_ransac(point_cloud)
            if plane_params is not None:
                self.get_logger().info(f'Found plane with parameters: {plane_params}')

        return depth_image

    def depth_to_point_cloud(self, depth_image):
        """
        Convert depth image to 3D point cloud using camera intrinsics
        """
        if self.camera_matrix is None:
            return np.array([])

        # Get image dimensions
        height, width = depth_image.shape

        # Create coordinate grids
        u_coords, v_coords = np.meshgrid(np.arange(width), np.arange(height))

        # Get depth values at each pixel
        z_coords = depth_image

        # Apply camera intrinsics to convert to 3D coordinates
        x_coords = (u_coords - self.camera_matrix[0, 2]) * z_coords / self.camera_matrix[0, 0]
        y_coords = (v_coords - self.camera_matrix[1, 2]) * z_coords / self.camera_matrix[1, 1]

        # Stack to create point cloud (filter out invalid points)
        valid_mask = (z_coords > 0) & (np.isfinite(x_coords)) & (np.isfinite(y_coords))

        points = np.stack([x_coords[valid_mask], y_coords[valid_mask], z_coords[valid_mask]], axis=1)

        return points

    def find_plane_ransac(self, points, num_iterations=100, threshold=0.05):
        """
        Find the best fitting plane in the point cloud using RANSAC
        """
        if len(points) < 3:
            return None

        best_inliers = 0
        best_plane = None

        for _ in range(num_iterations):
            # Randomly select 3 points
            random_indices = np.random.choice(len(points), 3, replace=False)
            sample_points = points[random_indices]

            # Calculate plane parameters from 3 points
            v1 = sample_points[1] - sample_points[0]
            v2 = sample_points[2] - sample_points[0]

            # Calculate normal vector (cross product)
            normal = np.cross(v1, v2)

            # Normalize the normal vector
            norm = np.linalg.norm(normal)
            if norm == 0:
                continue
            normal = normal / norm

            # Calculate d parameter for plane equation ax + by + cz + d = 0
            d = -np.dot(normal, sample_points[0])

            # Calculate distances from all points to the plane
            distances = np.abs(np.dot(points, normal) + d)

            # Count inliers
            inliers = np.sum(distances < threshold)

            # Update best plane if this one has more inliers
            if inliers > best_inliers:
                best_inliers = inliers
                best_plane = np.append(normal, d)

        return best_plane


def main(args=None):
    rclpy.init(args=args)

    node = DepthPerceptionDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Depth Perception Demo node')
    finally:
        # Clean up OpenCV windows
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()