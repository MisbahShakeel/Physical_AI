#!/usr/bin/env python3
"""
Vision-Language-Action Perception Node

This script implements a ROS 2 node that integrates visual perception
for the Vision-Language-Action control loop.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import String, Header
from geometry_msgs.msg import Point, Pose, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time
from typing import Dict, List, Any, Optional
import logging
from dataclasses import dataclass


@dataclass
class ObjectDetection:
    """Represents a detected object"""
    name: str
    confidence: float
    position: Point
    pose: Pose
    bounding_box: List[float]  # [x, y, width, height]


class VLAPerceptionNode(Node):
    """
    ROS 2 node for Vision-Language-Action perception
    """

    def __init__(self):
        super().__init__('vla_perception_node')

        # Initialize OpenCV bridge
        self.cv_bridge = CvBridge()

        # Internal state
        self.latest_image = None
        self.camera_info = None
        self.object_detections = []
        self.scene_description = ""
        self.last_detection_time = time.time()

        # Publishers
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/vla/detections',
            10
        )

        self.scene_publisher = self.create_publisher(
            String,
            '/vla/scene_description',
            10
        )

        self.perception_status_publisher = self.create_publisher(
            String,
            '/vla/perception_status',
            10
        )

        # Subscribers
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )

        # Timer for perception processing
        self.perception_timer = self.create_timer(
            0.5,  # 2 Hz
            self.perception_callback
        )

        # Perception parameters
        self.detection_confidence_threshold = 0.5
        self.max_detection_distance = 3.0  # meters
        self.object_classes = [
            'person', 'bottle', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana',
            'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
            'donut', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet',
            'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors'
        ]

        self.get_logger().info('VLA Perception node initialized')

    def image_callback(self, msg: Image):
        """
        Handle incoming camera images
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        """
        Handle camera info messages
        """
        self.camera_info = msg

    def perception_callback(self):
        """
        Main perception processing callback
        """
        if self.latest_image is None:
            return

        try:
            # Process the latest image for object detection
            detections = self._detect_objects(self.latest_image)

            # Update internal state
            self.object_detections = detections
            self.scene_description = self._describe_scene(detections)

            # Publish detections
            self._publish_detections(detections)

            # Publish scene description
            scene_msg = String()
            scene_msg.data = self.scene_description
            self.scene_publisher.publish(scene_msg)

            # Publish status
            status_msg = String()
            status_msg.data = f"DETECTED: {len(detections)} objects"
            self.perception_status_publisher.publish(status_msg)

            self.last_detection_time = time.time()

        except Exception as e:
            self.get_logger().error(f'Error in perception callback: {e}')

    def _detect_objects(self, image: np.ndarray) -> List[ObjectDetection]:
        """
        Detect objects in the image (mock implementation for demo)

        Args:
            image: Input image in OpenCV format

        Returns:
            List of detected objects
        """
        # In a real implementation, this would use a deep learning model like YOLO or Detectron2
        # For demo purposes, we'll simulate object detection

        # Simulate processing delay
        time.sleep(0.1)

        # Generate mock detections based on image properties
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height // 2

        # Create some mock detections
        mock_objects = [
            ObjectDetection(
                name='red_cube',
                confidence=0.85,
                position=Point(x=center_x - 50, y=center_y, z=0.0),
                pose=Pose(),
                bounding_box=[center_x - 60, center_y - 10, 20, 20]
            ),
            ObjectDetection(
                name='blue_sphere',
                confidence=0.78,
                position=Point(x=center_x + 50, y=center_y, z=0.0),
                pose=Pose(),
                bounding_box=[center_x + 40, center_y - 10, 20, 20]
            ),
            ObjectDetection(
                name='green_cylinder',
                confidence=0.92,
                position=Point(x=center_x, y=center_y + 50, z=0.0),
                pose=Pose(),
                bounding_box=[center_x - 10, center_y + 40, 20, 20]
            )
        ]

        # Filter based on confidence threshold
        filtered_objects = [
            obj for obj in mock_objects
            if obj.confidence >= self.detection_confidence_threshold
        ]

        self.get_logger().debug(f'Detected {len(filtered_objects)} objects')

        return filtered_objects

    def _describe_scene(self, detections: List[ObjectDetection]) -> str:
        """
        Generate a textual description of the scene

        Args:
            detections: List of detected objects

        Returns:
            Scene description as string
        """
        if not detections:
            return "No objects detected in the scene."

        # Create a scene description
        object_names = [det.name for det in detections]
        unique_names = list(set(object_names))

        description = f"The scene contains {len(detections)} objects: "
        description += ", ".join(unique_names)

        # Add spatial relationships
        if len(detections) >= 2:
            # Find the two most confident detections
            sorted_detections = sorted(detections, key=lambda x: x.confidence, reverse=True)
            if len(sorted_detections) >= 2:
                obj1 = sorted_detections[0]
                obj2 = sorted_detections[1]

                # Calculate relative position (simplified)
                dx = obj2.position.x - obj1.position.x
                dy = obj2.position.y - obj1.position.y

                if abs(dx) > abs(dy):  # Horizontal relationship is stronger
                    if dx > 0:
                        relationship = f"{obj2.name} is to the right of {obj1.name}"
                    else:
                        relationship = f"{obj2.name} is to the left of {obj1.name}"
                else:  # Vertical relationship is stronger
                    if dy > 0:
                        relationship = f"{obj2.name} is below {obj1.name}"
                    else:
                        relationship = f"{obj2.name} is above {obj1.name}"

                description += f". {relationship}."

        return description

    def _publish_detections(self, detections: List[ObjectDetection]):
        """
        Publish object detections to ROS 2 topic

        Args:
            detections: List of detected objects to publish
        """
        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'camera_rgb_optical_frame'

        for detection in detections:
            # Create a vision_msgs/Detection2D object
            vision_detection = ObjectHypothesisWithPose()
            vision_detection.hypothesis.class_id = detection.name
            vision_detection.hypothesis.score = detection.confidence

            # Add to array
            detection_msg = Detection2D()
            detection_msg.header = detection_array.header
            detection_msg.results = [vision_detection]

            # Set bounding box (in image coordinates)
            detection_msg.bbox.center.x = detection.bounding_box[0] + detection.bounding_box[2] / 2
            detection_msg.bbox.center.y = detection.bounding_box[1] + detection.bounding_box[3] / 2
            detection_msg.bbox.size_x = detection.bounding_box[2]
            detection_msg.bbox.size_y = detection.bounding_box[3]

            detection_array.detections.append(detection_msg)

        self.detection_publisher.publish(detection_array)

    def get_current_scene_state(self) -> Dict[str, Any]:
        """
        Get the current state of the scene as a dictionary

        Returns:
            Dictionary containing scene information
        """
        return {
            'timestamp': time.time(),
            'objects': [
                {
                    'name': obj.name,
                    'confidence': obj.confidence,
                    'position': {
                        'x': obj.position.x,
                        'y': obj.position.y,
                        'z': obj.position.z
                    },
                    'bbox': obj.bounding_box
                }
                for obj in self.object_detections
            ],
            'scene_description': self.scene_description,
            'detection_count': len(self.object_detections)
        }

    def get_object_by_name(self, name: str) -> Optional[ObjectDetection]:
        """
        Get a specific object by name

        Args:
            name: Name of the object to find

        Returns:
            ObjectDetection if found, None otherwise
        """
        for obj in self.object_detections:
            if obj.name.lower() == name.lower():
                return obj
        return None


def main(args=None):
    """
    Main function to run the VLA perception node
    """
    rclpy.init(args=args)

    node = VLAPerceptionNode()

    try:
        node.get_logger().info("VLA Perception node starting...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down VLA Perception node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()