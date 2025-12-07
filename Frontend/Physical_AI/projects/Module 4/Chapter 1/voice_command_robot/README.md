# Voice Command Robot Project

This project implements a voice command system for a simulated humanoid robot, converting speech to simple actions like "move forward", "turn left", etc.

## Project Structure

- `voice_robot_controller.py` - Main controller that processes voice commands and executes robot actions
- `command_parser.py` - Natural language processing for voice command interpretation
- `robot_simulator.py` - Simulated humanoid robot with basic movement capabilities
- `voice_interface.py` - Integration with voice-to-text system
- `config.yaml` - Configuration parameters for the voice command system

## Components

### Voice Command Processing
- Audio capture and preprocessing
- Speech-to-text conversion using OpenAI Whisper
- Natural language understanding for command interpretation
- Command validation and safety checks

### Robot Action Execution
- Mapping voice commands to robot movements
- Safety validation before action execution
- Feedback mechanisms for command confirmation
- Error handling for unrecognized commands

### Command Vocabulary
- Basic movement commands: forward, backward, left, right, turn
- Action commands: stop, dance, wave, etc.
- Context-aware command interpretation

## Running the Project

1. Set up your OpenAI API key as an environment variable:
   ```bash
   export OPENAI_API_KEY='your-api-key-here'
   ```

2. Run the main controller:
   ```bash
   python voice_robot_controller.py
   ```

3. Speak commands to your microphone, such as:
   - "Move forward"
   - "Turn left"
   - "Stop"
   - "Dance"

## Key Concepts Demonstrated

- Voice-to-action conversion for humanoid robots
- Natural language processing for command interpretation
- Integration of speech recognition with robot control
- Safety mechanisms for voice-controlled robots
- Feedback systems for voice command confirmation