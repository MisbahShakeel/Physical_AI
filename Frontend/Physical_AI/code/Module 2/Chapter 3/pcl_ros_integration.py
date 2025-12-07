#!/usr/bin/env python3
"""
PCL-ROS 2 Integration Example (Conceptual)

This script demonstrates the concepts of integrating PCL with ROS 2 for point cloud processing.
Note: Python PCL bindings are limited, so this example shows the conceptual approach
and would typically be implemented in C++ in practice.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np


class PCLROSIntegration(Node):
    def __init__(self):
        super().__init__('pcl_ros_integration')

        # Subscribe to point cloud topic
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',  # Replace with your point cloud topic
            self.pointcloud_callback,
            10
        )

        # Publisher for processed point cloud (filtered)
        self.filtered_cloud_publisher = self.create_publisher(
            PointCloud2,
            '/camera/depth/points_filtered',
            10
        )

        self.get_logger().info('PCL-ROS Integration node started')

    def pointcloud_callback(self, msg):
        """Process incoming point cloud message"""
        try:
            # Convert ROS PointCloud2 message to list of points
            points_list = []
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points_list.append([point[0], point[1], point[2]])

            if points_list:
                # Convert to numpy array for processing
                points_array = np.array(points_list)

                # Apply basic filtering (remove points too far away)
                filtered_points = self.filter_point_cloud(points_array)

                # Create new PointCloud2 message with filtered data
                filtered_cloud_msg = self.create_pointcloud2_msg(filtered_points, msg.header)

                # Publish the filtered point cloud
                self.filtered_cloud_publisher.publish(filtered_cloud_msg)

                self.get_logger().debug(f'Processed point cloud: {len(points_list)} -> {len(filtered_points)} points')

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')

    def filter_point_cloud(self, points):
        """
        Apply basic filtering to the point cloud:
        - Remove points beyond a certain distance
        - Remove points below a certain distance (remove noise)
        """
        # Define filtering parameters
        min_distance = 0.1  # meters
        max_distance = 5.0  # meters

        # Calculate distances from origin
        distances = np.sqrt(np.sum(points**2, axis=1))

        # Filter points based on distance
        valid_indices = (distances >= min_distance) & (distances <= max_distance)
        filtered_points = points[valid_indices]

        return filtered_points

    def create_pointcloud2_msg(self, points, header):
        """
        Create a PointCloud2 message from numpy array of points
        """
        # Create field definitions for x, y, z coordinates
        fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1)
        ]

        # Convert numpy array to list of tuples for point_cloud2.create_cloud
        points_tuple = [tuple(point) for point in points]

        # Create the PointCloud2 message
        cloud_msg = point_cloud2.create_cloud(header, fields, points_tuple)

        return cloud_msg


def main(args=None):
    rclpy.init(args=args)

    node = PCLROSIntegration()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down PCL-ROS Integration node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()