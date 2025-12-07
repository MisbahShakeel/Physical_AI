# AI Integration for Robot Decision Making

This project demonstrates integration of AI models (LLMs and VLMs) with ROS 2 for intelligent robot decision making and behavior.

## Components

- `llm_ros_integration.py`: Example of integrating Large Language Models with ROS 2 for natural language command processing
- `vlm_ros_integration.py`: Example of Vision-Language Model integration with ROS 2 for multimodal perception
- `ai_decision_maker.py`: Complete AI-driven decision making system that combines LLM and VLM capabilities
- `README.md`: This documentation file

## How it Works

The AI decision maker integrates multiple AI capabilities:

1. **Natural Language Understanding**: Processes human commands using LLM-like reasoning
2. **Vision-Language Processing**: Analyzes visual scenes and responds to queries using VLM-like capabilities
3. **Decision Making**: Combines perception and language understanding to make intelligent decisions
4. **Action Execution**: Translates high-level decisions into robot actions

## Running the Examples

1. Make sure you have ROS 2 and required dependencies installed
2. Launch a robot simulation with camera and LiDAR sensors
3. Run the LLM integration example:
   ```bash
   python3 llm_ros_integration.py
   ```

4. Run the VLM integration example:
   ```bash
   python3 vlm_ros_integration.py
   ```

5. Run the complete AI decision maker:
   ```bash
   python3 ai_decision_maker.py
   ```

## Key Concepts Demonstrated

- **LLM Integration**: Natural language command processing and task planning
- **VLM Integration**: Vision-language understanding and scene analysis
- **Multimodal Perception**: Combining visual and linguistic information
- **AI Decision Making**: Intelligent behavior generation from high-level commands
- **ROS 2 Integration**: Proper integration patterns for AI-robot systems

## Expected Behavior

The AI decision maker will:
- Process natural language commands from users
- Analyze visual scenes using vision-language capabilities
- Make intelligent decisions based on both language and perception
- Execute appropriate robot behaviors
- Provide natural language responses to users