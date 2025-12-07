#!/usr/bin/env python3
"""
Voice-to-Text ROS 2 Node Using OpenAI Whisper

This script implements a ROS 2 node that captures audio and publishes
transcribed text using OpenAI Whisper API.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import AudioData
from geometry_msgs.msg import Twist
import numpy as np
import sounddevice as sd
import soundfile as sf
import tempfile
import os
import threading
import time
from typing import Optional, Dict, Any
import logging
import json


class VoiceToTextNode(Node):
    """
    ROS 2 node for converting voice commands to text using OpenAI Whisper
    """

    def __init__(self):
        super().__init__('voice_to_text_node')

        # Initialize Whisper transcriber (using mock implementation for demo)
        self.transcriber = MockWhisperTranscriber()

        # Audio recording parameters
        self.sample_rate = 16000
        self.channels = 1
        self.chunk_duration = 2.0  # seconds
        self.recording = False
        self.recording_thread = None

        # Confidence threshold for valid transcriptions
        self.confidence_threshold = 0.7

        # Publishers
        self.text_publisher = self.create_publisher(
            String,
            '/voice_commands',
            10
        )

        self.confidence_publisher = self.create_publisher(
            Float32,
            '/voice_confidence',
            10
        )

        self.status_publisher = self.create_publisher(
            String,
            '/voice_status',
            10
        )

        # Subscribers
        self.command_subscription = self.create_subscription(
            String,
            '/voice_control',
            self.voice_control_callback,
            10
        )

        # Timer for continuous recording
        self.recording_timer = self.create_timer(
            self.chunk_duration,
            self.recording_callback
        )

        # Internal state
        self.last_transcription = ""
        self.is_active = True

        self.get_logger().info('Voice-to-Text node initialized')
        self.publish_status("READY")

    def voice_control_callback(self, msg: String):
        """
        Handle voice control commands
        """
        command = msg.data.lower()

        if command == "start":
            self.is_active = True
            self.publish_status("ACTIVE")
            self.get_logger().info("Voice recognition activated")

        elif command == "stop":
            self.is_active = False
            self.publish_status("INACTIVE")
            self.get_logger().info("Voice recognition deactivated")

        elif command == "test":
            self.get_logger().info("Running voice recognition test")
            self.test_transcription()

        else:
            self.get_logger().warn(f"Unknown voice control command: {command}")

    def recording_callback(self):
        """
        Timer callback for continuous audio recording and transcription
        """
        if not self.is_active:
            return

        try:
            # Record audio chunk
            audio_data = sd.rec(
                int(self.chunk_duration * self.sample_rate),
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype='float32'
            )
            sd.wait()  # Wait for recording to complete

            # Transcribe the audio
            result = self.transcriber.transcribe_audio_data(audio_data.flatten())

            if result and result['confidence'] >= self.confidence_threshold:
                # Publish the transcribed text
                text_msg = String()
                text_msg.data = result['text']
                self.text_publisher.publish(text_msg)

                # Publish confidence
                conf_msg = Float32()
                conf_msg.data = result['confidence']
                self.confidence_publisher.publish(conf_msg)

                self.last_transcription = result['text']
                self.get_logger().info(f"Transcribed: '{result['text']}' (conf: {result['confidence']:.2f})")

                # Publish status
                status_msg = String()
                status_msg.data = f"TRANSCRIBED: {result['text'][:50]}..."
                self.status_publisher.publish(status_msg)

            elif result:
                self.get_logger().debug(f"Low confidence transcription ignored: {result['confidence']:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error in recording callback: {e}")

    def test_transcription(self):
        """
        Test transcription with sample audio
        """
        self.get_logger().info("Testing transcription with sample data")

        # Generate sample audio data (simulated)
        sample_audio = np.random.random(int(2.0 * self.sample_rate)).astype(np.float32)

        try:
            result = self.transcriber.transcribe_audio_data(sample_audio)

            if result:
                self.get_logger().info(f"Test transcription: {result['text']}")

                # Publish test result
                text_msg = String()
                text_msg.data = result['text']
                self.text_publisher.publish(text_msg)

                conf_msg = Float32()
                conf_msg.data = result['confidence']
                self.confidence_publisher.publish(conf_msg)

        except Exception as e:
            self.get_logger().error(f"Error in test transcription: {e}")

    def publish_status(self, status: str):
        """
        Publish status message
        """
        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)


class MockWhisperTranscriber:
    """
    Mock implementation of Whisper transcriber for demonstration
    In a real implementation, this would interface with the OpenAI Whisper API
    """

    def __init__(self):
        self.commands = [
            "move forward",
            "turn left",
            "turn right",
            "stop moving",
            "raise your hand",
            "lower your hand",
            "dance",
            "hello",
            "goodbye",
            "help me",
            "come here",
            "go there",
            "pick up object",
            "put down object"
        ]
        self.command_index = 0

    def transcribe_audio_data(self, audio_data: np.ndarray) -> Optional[Dict[str, Any]]:
        """
        Mock transcription of audio data
        """
        # Simulate processing delay
        time.sleep(0.1)

        # For demo purposes, cycle through predefined commands
        # In a real implementation, this would call the Whisper API
        command = self.commands[self.command_index % len(self.commands)]
        self.command_index += 1

        # Simulate confidence score
        confidence = 0.7 + np.random.random() * 0.3  # Between 0.7 and 1.0

        result = {
            'text': command,
            'confidence': confidence,
            'language': 'en',
            'duration': len(audio_data) / 16000.0
        }

        return result

    def transcribe_audio_file(self, audio_filepath: str) -> Optional[Dict[str, Any]]:
        """
        Mock transcription of audio file
        """
        # In a real implementation, this would read the file and call Whisper API
        return self.transcribe_audio_data(np.random.random(1000).astype(np.float32))


def main(args=None):
    """
    Main function to run the voice-to-text node
    """
    rclpy.init(args=args)

    node = VoiceToTextNode()

    try:
        node.get_logger().info("Voice-to-Text node starting...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Voice-to-Text node")
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()