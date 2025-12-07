# Human-Robot Interaction and Social Cognition

This project demonstrates human-robot interaction (HRI) and social cognition capabilities for humanoid robots, including social signal processing, theory of mind, and trust building mechanisms.

## Components

- `social_interaction_core.py`: Core social interaction system with theory of mind and social decision making
- `social_signal_processing.py`: Social signal processing for gesture, gaze, and emotion recognition
- `trust_building_demo.py`: Complete demonstration of trust building and social adaptation
- `README.md`: This documentation file

## How it Works

The social interaction system implements several key capabilities:

1. **Social Signal Processing**: Recognizes gestures, tracks gaze, detects emotions, and analyzes proxemics
2. **Theory of Mind**: Attributes mental states to humans and predicts intentions
3. **Social Decision Making**: Makes decisions considering human preferences and social norms
4. **Trust Building**: Builds and maintains trust through consistent, reliable interactions
5. **Social Adaptation**: Adapts behavior based on individual preferences and trust levels

## Running the Examples

1. Make sure you have ROS 2 and required dependencies installed
2. Launch a robot simulation with camera and LiDAR sensors
3. Run the core social interaction system:
   ```bash
   python3 social_interaction_core.py
   ```

4. Run the social signal processing system:
   ```bash
   python3 social_signal_processing.py
   ```

5. Run the trust building demonstration:
   ```bash
   python3 trust_building_demo.py
   ```

## Key Concepts Demonstrated

- **Social Signal Processing**: Multimodal recognition of human social cues
- **Theory of Mind**: Understanding human beliefs, desires, and intentions
- **Emotion Recognition**: Detecting and responding to human emotions
- **Trust Building**: Mechanisms for establishing and maintaining human trust
- **Social Adaptation**: Personalizing interaction based on individual preferences
- **Proxemics**: Understanding and respecting personal space and social distances
- **Social Decision Making**: Considering social factors in robot decision-making

## Expected Behavior

The social interaction system will:
- Detect humans and recognize their gestures, gaze, and emotions
- Attribute mental states and predict human intentions
- Make socially appropriate decisions based on context
- Build trust through consistent and helpful interactions
- Adapt behavior based on individual preferences and trust levels
- Maintain appropriate social distances and interaction styles
- Provide natural language responses that reflect social awareness