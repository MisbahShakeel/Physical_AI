#!/usr/bin/env python3
"""
OpenAI Whisper Audio Transcription Script

This script implements audio-to-text transcription using OpenAI Whisper API
for voice command processing in humanoid robots.
"""

import os
import openai
import numpy as np
import sounddevice as sd
import soundfile as sf
import io
import wave
import tempfile
import requests
import json
from typing import Optional, Dict, Any, List
import logging
import time
import threading
from pathlib import Path


class WhisperTranscriber:
    """
    A class to handle audio transcription using OpenAI Whisper
    """

    def __init__(self, api_key: Optional[str] = None, model: str = "whisper-1"):
        """
        Initialize the Whisper transcriber

        Args:
            api_key: OpenAI API key (if not set, looks for OPENAI_API_KEY environment variable)
            model: Whisper model to use (default: whisper-1)
        """
        # Set API key
        if api_key:
            openai.api_key = api_key
        elif os.getenv("OPENAI_API_KEY"):
            openai.api_key = os.getenv("OPENAI_API_KEY")
        else:
            # For demonstration purposes, we'll use a mock implementation
            logging.warning("No OpenAI API key found. Using mock implementation.")
            self.use_mock = True
        self.use_mock = False  # Set to False when using real API

        self.model = model
        self.sample_rate = 16000  # Standard for speech recognition
        self.channels = 1         # Mono audio
        self.recording = False
        self.audio_buffer = []

        # Set up logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

    def record_audio(self, duration: float = 5.0) -> np.ndarray:
        """
        Record audio from the microphone

        Args:
            duration: Recording duration in seconds

        Returns:
            Audio data as numpy array
        """
        self.logger.info(f"Recording audio for {duration} seconds...")

        # Record audio
        audio_data = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=self.channels,
            dtype='float32'
        )
        sd.wait()  # Wait for recording to complete

        self.logger.info("Audio recording completed")
        return audio_data.flatten()

    def save_audio_to_wav(self, audio_data: np.ndarray, filepath: str) -> str:
        """
        Save audio data to a WAV file

        Args:
            audio_data: Audio data as numpy array
            filepath: Path to save the WAV file

        Returns:
            Path to the saved file
        """
        sf.write(filepath, audio_data, self.sample_rate)
        self.logger.info(f"Audio saved to {filepath}")
        return filepath

    def transcribe_audio_file(self, audio_filepath: str) -> Dict[str, Any]:
        """
        Transcribe audio file using OpenAI Whisper API

        Args:
            audio_filepath: Path to the audio file to transcribe

        Returns:
            Transcription result as dictionary
        """
        if self.use_mock:
            # Mock implementation for demonstration
            self.logger.info(f"Mock transcription of {audio_filepath}")
            # Simulate transcription delay
            time.sleep(0.5)
            # Return mock result
            return {
                "text": "move forward ten steps",
                "confidence": 0.9,
                "language": "en",
                "duration": 2.5
            }
        else:
            # Real implementation using OpenAI API
            try:
                with open(audio_filepath, "rb") as audio_file:
                    transcript = openai.Audio.transcribe(
                        model=self.model,
                        file=audio_file,
                        response_format="verbose_json",
                        timestamp_granularities=["segment"]
                    )

                # Calculate confidence based on the transcription
                confidence = self._calculate_confidence(transcript)

                result = {
                    "text": transcript.text,
                    "confidence": confidence,
                    "language": transcript.language,
                    "duration": transcript.duration
                }

                self.logger.info(f"Transcription completed: {result['text'][:50]}...")
                return result

            except Exception as e:
                self.logger.error(f"Transcription error: {e}")
                raise

    def transcribe_audio_data(self, audio_data: np.ndarray) -> Dict[str, Any]:
        """
        Transcribe raw audio data using OpenAI Whisper API

        Args:
            audio_data: Audio data as numpy array

        Returns:
            Transcription result as dictionary
        """
        # Create a temporary WAV file
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
            temp_path = temp_file.name
            self.save_audio_to_wav(audio_data, temp_path)

            try:
                result = self.transcribe_audio_file(temp_path)
                return result
            finally:
                # Clean up temporary file
                os.unlink(temp_path)

    def _calculate_confidence(self, transcript: Any) -> float:
        """
        Calculate confidence score from transcript

        Args:
            transcript: Transcript object from OpenAI API

        Returns:
            Confidence score between 0 and 1
        """
        # In a real implementation, this would analyze the transcript for confidence
        # For now, we'll return a fixed high confidence for demonstration
        return 0.9

    def continuous_transcription(self, callback_func=None, max_duration: float = 10.0):
        """
        Perform continuous audio transcription with a callback function

        Args:
            callback_func: Function to call with each transcription result
            max_duration: Maximum duration for continuous recording
        """
        self.logger.info("Starting continuous transcription...")

        try:
            audio_data = self.record_audio(duration=max_duration)
            result = self.transcribe_audio_data(audio_data)

            if callback_func:
                callback_func(result)

            return result

        except Exception as e:
            self.logger.error(f"Continuous transcription error: {e}")
            raise

    def transcribe_with_confidence_threshold(self, audio_data: np.ndarray,
                                           threshold: float = 0.7) -> Optional[Dict[str, Any]]:
        """
        Transcribe audio with confidence threshold filtering

        Args:
            audio_data: Audio data as numpy array
            threshold: Minimum confidence threshold (0.0 to 1.0)

        Returns:
            Transcription result if confidence >= threshold, else None
        """
        result = self.transcribe_audio_data(audio_data)

        if result["confidence"] >= threshold:
            return result
        else:
            self.logger.info(f"Transcription confidence {result['confidence']} below threshold {threshold}")
            return None


def main():
    """
    Main function to demonstrate Whisper transcription capabilities
    """
    print("OpenAI Whisper Transcription Demo")
    print("="*40)

    # Initialize transcriber
    # For demo purposes, we'll use mock mode
    transcriber = WhisperTranscriber()

    # Option 1: Record and transcribe audio
    print("\n1. Recording audio...")
    try:
        audio_data = transcriber.record_audio(duration=3.0)
        print(f"Recorded {len(audio_data)} audio samples")

        print("\n2. Transcribing audio...")
        result = transcriber.transcribe_audio_data(audio_data)

        print(f"\nTranscription result:")
        print(f"Text: {result['text']}")
        print(f"Confidence: {result['confidence']:.2f}")
        print(f"Language: {result['language']}")
        print(f"Duration: {result['duration']:.2f}s")

    except Exception as e:
        print(f"Error during recording/transcription: {e}")

    # Option 2: Continuous transcription demo
    print("\n" + "="*40)
    print("Continuous transcription demo (mock implementation)")

    def transcription_callback(result):
        print(f"Continuous result: {result['text']} (conf: {result['confidence']:.2f})")

    try:
        result = transcriber.continuous_transcription(
            callback_func=transcription_callback,
            max_duration=2.0
        )
        print(f"Continuous transcription result: {result['text']}")
    except Exception as e:
        print(f"Error during continuous transcription: {e}")

    print("\nDemo completed!")


if __name__ == "__main__":
    main()