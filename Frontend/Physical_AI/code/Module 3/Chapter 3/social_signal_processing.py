#!/usr/bin/env python3
"""
Social Signal Processing and Emotion Recognition System

This script implements social signal processing capabilities for humanoid robots,
including gesture recognition, gaze tracking, and emotion detection from multiple modalities.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PointStamped, Vector3
from cv_bridge import CvBridge
import numpy as np
import time
import math
from typing import Dict, List, Tuple
import json
from enum import Enum


class EmotionState(Enum):
    NEUTRAL = "neutral"
    HAPPY = "happy"
    SAD = "sad"
    ANGRY = "angry"
    SURPRISED = "surprised"
    FEARFUL = "fearful"
    DISGUSTED = "disgusted"


class SocialSignalType(Enum):
    GESTURE = "gesture"
    GAZE = "gaze"
    PROXEMICS = "proxemics"
    VOCAL = "vocal"
    FACIAL = "facial"


class GestureRecognizer:
    """Recognizes human gestures from visual and spatial data"""

    def __init__(self):
        self.gesture_templates = {
            'wave': [(0, 0), (1, 0), (0, 0), (-1, 0), (0, 0)],  # Simple wave pattern
            'point': [(0, 0), (0.8, 0.2)],  # Pointing gesture
            'beckon': [(0, 0), (-0.5, 0.3), (0.5, -0.3)],  # Come here gesture
            'stop': [(0, 0), (0, 1), (0, 0)]  # Stop gesture
        }
        self.current_gesture_sequence = []
        self.gesture_threshold = 0.7

    def recognize_gesture(self, hand_positions: List[Tuple[float, float]]) -> Tuple[str, float]:
        """Recognize gesture from hand position sequence"""
        if len(hand_positions) < 2:
            return 'none', 0.0

        # Normalize the gesture sequence
        normalized_sequence = self._normalize_sequence(hand_positions)

        best_match = 'none'
        best_score = 0.0

        for template_name, template in self.gesture_templates.items():
            score = self._compare_sequences(normalized_sequence, template)
            if score > best_score:
                best_score = score
                best_match = template_name

        return best_match, best_score

    def _normalize_sequence(self, sequence: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Normalize gesture sequence for comparison"""
        if not sequence:
            return []

        # Convert to numpy array for easier manipulation
        seq_array = np.array(sequence)

        # Translate to start at origin
        seq_array = seq_array - seq_array[0]

        # Normalize to unit size
        max_dist = np.max(np.abs(seq_array))
        if max_dist > 0:
            seq_array = seq_array / max_dist

        return [tuple(point) for point in seq_array]

    def _compare_sequences(self, seq1: List[Tuple[float, float]], seq2: List[Tuple[float, float]]) -> float:
        """Compare two gesture sequences and return similarity score"""
        # Use dynamic time warping or simple distance comparison
        min_len = min(len(seq1), len(seq2))
        if min_len == 0:
            return 0.0

        distances = []
        for i in range(min_len):
            dist = math.sqrt((seq1[i][0] - seq2[i][0])**2 + (seq1[i][1] - seq2[i][1])**2)
            distances.append(dist)

        avg_distance = sum(distances) / len(distances)
        return max(0.0, 1.0 - avg_distance)  # Convert distance to similarity


class GazeTracker:
    """Tracks human gaze direction and attention focus"""

    def __init__(self):
        self.gaze_history = []
        self.attention_targets = {}
        self.smoothing_factor = 0.7

    def track_gaze(self, eye_positions: Tuple[float, float, float],
                   head_orientation: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Track gaze direction based on eye and head position"""
        # Calculate gaze direction from eye position and head orientation
        # Simplified calculation
        gaze_x = eye_positions[0] + head_orientation[0] * 0.1
        gaze_y = eye_positions[1] + head_orientation[1] * 0.1
        gaze_z = eye_positions[2] + head_orientation[2] * 0.1

        # Apply smoothing
        if self.gaze_history:
            last_gaze = self.gaze_history[-1]
            smoothed_gaze = (
                self.smoothing_factor * gaze_x + (1 - self.smoothing_factor) * last_gaze[0],
                self.smoothing_factor * gaze_y + (1 - self.smoothing_factor) * last_gaze[1],
                self.smoothing_factor * gaze_z + (1 - self.smoothing_factor) * last_gaze[2]
            )
        else:
            smoothed_gaze = (gaze_x, gaze_y, gaze_z)

        self.gaze_history.append(smoothed_gaze)
        if len(self.gaze_history) > 10:  # Keep last 10 gaze points
            self.gaze_history.pop(0)

        return smoothed_gaze

    def detect_joint_attention(self, robot_gaze: Tuple[float, float, float],
                              human_gaze: Tuple[float, float, float]) -> bool:
        """Detect if human and robot are looking at the same thing"""
        # Calculate angle between gazes
        dot_product = (robot_gaze[0] * human_gaze[0] +
                      robot_gaze[1] * human_gaze[1] +
                      robot_gaze[2] * human_gaze[2])

        # Normalize vectors
        robot_mag = math.sqrt(robot_gaze[0]**2 + robot_gaze[1]**2 + robot_gaze[2]**2)
        human_mag = math.sqrt(human_gaze[0]**2 + human_gaze[1]**2 + human_gaze[2]**2)

        if robot_mag > 0 and human_mag > 0:
            cos_angle = dot_product / (robot_mag * human_mag)
            # If angle is small (cosine close to 1), they're looking in similar direction
            return cos_angle > 0.8  # 36 degrees threshold

        return False


class EmotionRecognizer:
    """Recognizes human emotions from facial expressions and other cues"""

    def __init__(self):
        self.emotion_weights = {
            'happy': {'smile': 0.9, 'eyebrow': 0.3, 'gaze': 0.2},
            'sad': {'smile': -0.7, 'eyebrow': -0.8, 'gaze': -0.4},
            'angry': {'smile': -0.8, 'eyebrow': 0.9, 'gaze': 0.6},
            'surprised': {'smile': 0.2, 'eyebrow': 0.9, 'gaze': 0.7}
        }
        self.confidence_threshold = 0.6

    def recognize_emotion(self, facial_features: Dict) -> Tuple[EmotionState, float]:
        """Recognize emotion from facial features"""
        scores = {}

        for emotion, weights in self.emotion_weights.items():
            score = 0.0
            total_weight = 0.0

            for feature, weight in weights.items():
                if feature in facial_features:
                    score += weight * facial_features[feature]
                    total_weight += abs(weight)

            if total_weight > 0:
                scores[emotion] = score / total_weight
            else:
                scores[emotion] = 0.0

        # Find emotion with highest score
        best_emotion = max(scores, key=scores.get)
        best_score = scores[best_emotion]

        # Convert to enum and return with confidence
        emotion_enum = EmotionState(best_emotion)
        confidence = max(0.0, min(1.0, abs(best_score)))

        return emotion_enum, confidence

    def get_emotion_appropriate_response(self, emotion: EmotionState) -> str:
        """Get appropriate response based on detected emotion"""
        responses = {
            EmotionState.HAPPY: "You seem happy! I'm glad to see you in good spirits.",
            EmotionState.SAD: "You seem sad. Is there anything I can do to help?",
            EmotionState.ANGRY: "I notice you might be upset. How can I assist you?",
            EmotionState.SURPRISED: "You seem surprised. Is everything okay?",
            EmotionState.FEARFUL: "You appear anxious. I'm here to help if needed.",
            EmotionState.DISGUSTED: "You seem uncomfortable. Can I adjust something?",
            EmotionState.NEUTRAL: "I see you. How can I assist you today?"
        }
        return responses.get(emotion, "Hello. How can I help you?")


class ProxemicsAnalyzer:
    """Analyzes spatial relationships and proxemic behavior"""

    def __init__(self):
        self.personal_space_radius = 0.8  # meters
        self.social_space_radius = 1.5   # meters
        self.public_space_radius = 3.5   # meters
        self.intimate_space_radius = 0.45  # meters

    def analyze_proxemic_behavior(self, human_positions: List[Tuple[float, float, float]],
                                 robot_position: Tuple[float, float, float]) -> Dict:
        """Analyze proxemic behavior and spatial relationships"""
        if not human_positions:
            return {}

        results = {}
        for i, human_pos in enumerate(human_positions):
            # Calculate distance between human and robot
            distance = math.sqrt(
                (human_pos[0] - robot_position[0])**2 +
                (human_pos[1] - robot_position[1])**2 +
                (human_pos[2] - robot_position[2])**2
            )

            # Determine spatial zone
            if distance <= self.intimate_space_radius:
                zone = 'intimate'
            elif distance <= self.personal_space_radius:
                zone = 'personal'
            elif distance <= self.social_space_radius:
                zone = 'social'
            else:
                zone = 'public'

            results[f'human_{i}'] = {
                'distance': distance,
                'zone': zone,
                'comfort_level': self._get_comfort_level(zone, distance)
            }

        return results

    def _get_comfort_level(self, zone: str, distance: float) -> float:
        """Get comfort level based on spatial zone"""
        if zone == 'intimate':
            return 0.2  # Very uncomfortable
        elif zone == 'personal':
            return 0.6  # Somewhat uncomfortable
        elif zone == 'social':
            return 0.9  # Comfortable
        else:
            return 0.7  # Public distance, somewhat comfortable


class SocialSignalProcessorNode(Node):
    """ROS 2 node for processing social signals"""

    def __init__(self):
        super().__init__('social_signal_processor')

        # Initialize social signal processing systems
        self.gesture_recognizer = GestureRecognizer()
        self.gaze_tracker = GazeTracker()
        self.emotion_recognizer = EmotionRecognizer()
        self.proxemics_analyzer = ProxemicsAnalyzer()
        self.cv_bridge = CvBridge()

        # Subscribe to sensor data
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/humanoid/laser_scan',
            self.lidar_callback,
            10
        )

        self.robot_pose_subscription = self.create_subscription(
            PointStamped,
            '/robot/position',
            self.robot_pose_callback,
            10
        )

        # Publishers
        self.social_signals_publisher = self.create_publisher(
            String, '/social/signals', 10
        )
        self.emotion_publisher = self.create_publisher(
            String, '/social/emotion', 10
        )
        self.response_publisher = self.create_publisher(
            String, '/robot/response', 10
        )

        # Timer for processing loop
        self.processing_timer = self.create_timer(0.1, self.processing_loop)

        # Robot state
        self.robot_position = (0.0, 0.0, 0.0)
        self.human_positions = []
        self.facial_features = {}
        self.social_signals = {
            'gesture': {'type': 'none', 'confidence': 0.0},
            'gaze': {'direction': (0, 0, 1), 'joint_attention': False},
            'emotion': {'state': EmotionState.NEUTRAL.value, 'confidence': 0.0},
            'proxemics': {}
        }

        self.get_logger().info('Social Signal Processor Node initialized')

    def image_callback(self, msg):
        """Process visual data for social signals"""
        try:
            # Simulate facial feature extraction and gesture recognition
            # In real implementation, this would use computer vision models

            # Simulate detected facial features
            self.facial_features = {
                'smile': np.random.uniform(-1, 1),  # -1 to 1 scale
                'eyebrow': np.random.uniform(-1, 1),
                'gaze': np.random.uniform(-1, 1)
            }

            # Simulate gesture recognition
            # In real system, this would track hand positions over time
            simulated_hand_positions = [(0, 0), (0.5, 0.1), (0.2, -0.1)]
            gesture_type, gesture_confidence = self.gesture_recognizer.recognize_gesture(simulated_hand_positions)

            self.social_signals['gesture'] = {
                'type': gesture_type,
                'confidence': gesture_confidence
            }

            # Simulate gaze tracking
            simulated_eye_pos = (0.1, 0.2, 0.3)
            simulated_head_orient = (0.0, 0.0, 1.0)
            gaze_direction = self.gaze_tracker.track_gaze(simulated_eye_pos, simulated_head_orient)

            self.social_signals['gaze'] = {
                'direction': gaze_direction,
                'joint_attention': False  # Would be computed with robot gaze
            }

            # Recognize emotion
            emotion_state, emotion_confidence = self.emotion_recognizer.recognize_emotion(self.facial_features)

            self.social_signals['emotion'] = {
                'state': emotion_state.value,
                'confidence': emotion_confidence
            }

            self.get_logger().debug(f'Detected gesture: {gesture_type} (conf: {gesture_confidence:.2f}), '
                                  f'emotion: {emotion_state.value} (conf: {emotion_confidence:.2f})')

        except Exception as e:
            self.get_logger().error(f'Error processing image for social signals: {e}')

    def lidar_callback(self, msg):
        """Process LiDAR data for proxemics"""
        human_positions = []

        # Process LiDAR data to detect humans
        for i, range_val in enumerate(msg.ranges):
            if 0.5 < range_val < 3.0:  # Potential human distance
                angle = msg.angle_min + i * msg.angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                human_positions.append((x, y, 0.0))  # Assume z=0 for ground level

        self.human_positions = human_positions

        # Analyze proxemic behavior
        proxemic_results = self.proxemics_analyzer.analyze_proxemic_behavior(
            human_positions, self.robot_position
        )

        self.social_signals['proxemics'] = proxemic_results

    def robot_pose_callback(self, msg):
        """Update robot position"""
        self.robot_position = (msg.point.x, msg.point.y, msg.point.z)

    def processing_loop(self):
        """Main processing loop for social signals"""
        # Publish social signals
        signals_msg = String()
        signals_msg.data = json.dumps(self.social_signals)
        self.social_signals_publisher.publish(signals_msg)

        # Publish emotion
        emotion_msg = String()
        emotion_msg.data = json.dumps({
            'state': self.social_signals['emotion']['state'],
            'confidence': self.social_signals['emotion']['confidence']
        })
        self.emotion_publisher.publish(emotion_msg)

        # Generate appropriate response based on social signals
        self._generate_social_response()

    def _generate_social_response(self):
        """Generate social response based on detected signals"""
        gesture = self.social_signals['gesture']
        emotion_state = self.social_signals['emotion']['state']
        proxemics = self.social_signals['proxemics']

        response = ""

        # Check for specific gestures
        if gesture['type'] != 'none' and gesture['confidence'] > 0.5:
            if gesture['type'] == 'wave':
                response = "I see you waving! Hello there!"
            elif gesture['type'] == 'point':
                response = "I notice you're pointing. Can you tell me more about what you're indicating?"
            elif gesture['type'] == 'beckon':
                response = "You seem to be beckoning me over. I'm coming!"
            elif gesture['type'] == 'stop':
                response = "I see the stop gesture. I'll pause here."

        # If no specific gesture but emotion detected
        elif response == "":
            emotion_enum = EmotionState(emotion_state)
            response = self.emotion_recognizer.get_emotion_appropriate_response(emotion_enum)

        # Add proxemic awareness if humans are nearby
        if proxemics:
            close_humans = [k for k, v in proxemics.items() if v['distance'] < 1.0]
            if close_humans and "personal" in [proxemics[k]['zone'] for k in close_humans]:
                response += " I'm aware of your personal space."

        if response:
            self.publish_response(response)

    def publish_response(self, response_text):
        """Publish AI response to user"""
        response_msg = String()
        response_msg.data = response_text
        self.response_publisher.publish(response_msg)
        self.get_logger().info(f'Social Response: {response_text}')


def main(args=None):
    rclpy.init(args=args)

    node = SocialSignalProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Social Signal Processor Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()