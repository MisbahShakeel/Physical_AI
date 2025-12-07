# AI-Robot Brain Architecture

This project demonstrates a comprehensive AI-robot brain architecture that integrates Large Language Models (LLMs), Vision-Language Models (VLMs), and humanoid control systems.

## Components

- `ai_robot_brain.py`: Core cognitive architecture integrating perception, memory, reasoning, and planning systems
- `attention_learning_system.py`: Advanced system with attention mechanisms and learning capabilities
- `ai_brain_demo.py`: Complete demonstration system with safety monitoring and executive control
- `README.md`: This documentation file

## How it Works

The AI-robot brain integrates multiple cognitive systems:

1. **Perception System**: Processes multimodal sensory input (vision, LiDAR, IMU)
2. **Memory System**: Stores and retrieves episodic, semantic, and procedural knowledge
3. **Reasoning System**: Interprets commands and makes decisions using AI models
4. **Planning System**: Generates task and motion plans for robot execution
5. **Attention System**: Allocates computational resources efficiently
6. **Learning System**: Adapts behavior based on experience and feedback
7. **Safety System**: Monitors constraints and ensures safe operation

## Running the Examples

1. Make sure you have ROS 2 and required dependencies installed
2. Launch a robot simulation with camera, LiDAR, and IMU sensors
3. Run the basic AI brain:
   ```bash
   python3 ai_robot_brain.py
   ```

4. Run the advanced system with attention and learning:
   ```bash
   python3 attention_learning_system.py
   ```

5. Run the complete demonstration:
   ```bash
   python3 ai_brain_demo.py
   ```

## Key Concepts Demonstrated

- **Cognitive Architecture**: Unified framework for AI-robot integration
- **Multimodal Perception**: Integration of visual, spatial, and linguistic inputs
- **Attention Mechanisms**: Efficient resource allocation for real-time processing
- **Learning Systems**: Adaptive behavior improvement through experience
- **Safety Integration**: Constraint monitoring and emergency responses
- **Executive Control**: High-level task coordination and management

## Expected Behavior

The AI-robot brain will:
- Process natural language commands and interpret them using reasoning
- Integrate sensory information from multiple modalities
- Plan and execute appropriate robot behaviors
- Learn from experience to improve performance
- Allocate attention efficiently based on task demands
- Maintain safety constraints during operation
- Provide natural language responses to users