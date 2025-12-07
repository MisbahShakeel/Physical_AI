[Diagram: Voice-to-Action pipeline with OpenAI Whisper and ROS 2]

This diagram illustrates the complete voice-to-action pipeline for humanoid robots:

1. **Audio Input**:
   - Microphone array on humanoid robot
   - Audio preprocessing and noise reduction
   - Voice activity detection

2. **Speech Recognition**:
   - OpenAI Whisper API integration
   - Audio-to-text transcription
   - Confidence scoring and validation

3. **Command Processing**:
   - Natural language understanding
   - Intent recognition and parsing
   - Command validation and safety checks

4. **ROS 2 Integration**:
   - Audio input publisher node
   - Voice-to-text processing node
   - Command interpretation node
   - Action execution nodes

5. **Robot Control**:
   - Joint command interfaces
   - Navigation and movement control
   - Manipulation and interaction systems

6. **Feedback Loop**:
   - Visual feedback (LEDs, screen, gestures)
   - Audio confirmation
   - Action execution status
   - Error handling and recovery

The pipeline demonstrates how spoken commands flow from the user through the robot's audio system, speech recognition, and command processing to result in robot actions, with feedback to confirm successful execution.