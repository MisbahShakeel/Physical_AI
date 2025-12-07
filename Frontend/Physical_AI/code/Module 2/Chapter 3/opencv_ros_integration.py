#!/usr/bin/env python3
"""
OpenCV-ROS 2 Integration Example

This script demonstrates how to integrate OpenCV with ROS 2 for real-time image processing.
It subscribes to camera image data, processes it using OpenCV, and publishes the results.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class OpenCVROSIntegration(Node):
    def __init__(self):
        super().__init__('opencv_ros_integration')

        # Create CvBridge instance for converting between ROS and OpenCV images
        self.bridge = CvBridge()

        # Subscribe to camera image topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',  # Replace with your camera topic
            self.image_callback,
            10
        )

        # Publisher for processed images (optional)
        self.processed_image_publisher = self.create_publisher(
            Image,
            '/camera/processed_image',
            10
        )

        self.get_logger().info('OpenCV-ROS Integration node started')

    def image_callback(self, msg):
        """Process incoming image message"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process the image using OpenCV
            processed_image = self.process_image(cv_image)

            # Optionally publish the processed image
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            self.processed_image_publisher.publish(processed_msg)

            # Display the original and processed images
            cv2.imshow('Original Image', cv_image)
            cv2.imshow('Processed Image', processed_image)
            cv2.waitKey(1)  # Needed for OpenCV GUI to update

            self.get_logger().debug('Image processed successfully')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def process_image(self, image):
        """
        Apply various OpenCV processing techniques to the image
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(image, (5, 5), 0)

        # Edge detection using Canny
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray_blur, 50, 150)

        # Color-based filtering (example: detect red objects)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        red_mask = cv2.bitwise_or(mask1, mask2)

        # Combine edge detection and color filtering
        combined = cv2.bitwise_and(image, image, mask=red_mask)
        combined = cv2.addWeighted(combined, 1, cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR), 0.3, 0)

        # Find contours of red objects
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours on the original image
        result = image.copy()
        cv2.drawContours(result, contours, -1, (0, 255, 0), 2)

        # Add text showing number of contours found
        cv2.putText(result, f'Red objects: {len(contours)}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        return result


def main(args=None):
    rclpy.init(args=args)

    node = OpenCVROSIntegration()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down OpenCV-ROS Integration node')
    finally:
        # Clean up OpenCV windows
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()