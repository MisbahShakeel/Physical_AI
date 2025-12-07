# Embodied AI and Situated Cognition

This project demonstrates embodied AI and situated cognition principles for humanoid robots, focusing on how robots can learn and adapt through physical interaction with their environment.

## Components

- `embodied_ai_core.py`: Core embodied AI system with body schema, affordance detection, and sensorimotor learning
- `affordance_perception.py`: Affordance-based perception and action selection system
- `situated_learning_demo.py`: Complete demonstration of situated learning through environmental interaction
- `README.md`: This documentation file

## How it Works

The embodied AI system operates on the principle that intelligence emerges from the dynamic interaction between:

1. **Body Schema**: The robot's representation of its own physical form and capabilities
2. **Environmental Perception**: Detection of affordances (action possibilities) in the environment
3. **Sensorimotor Learning**: Learning through physical interaction and experience
4. **Situated Context**: Environmental context that shapes perception and action

## Running the Examples

1. Make sure you have ROS 2 and required dependencies installed
2. Launch a robot simulation with camera, LiDAR, and joint state publishers
3. Run the core embodied AI system:
   ```bash
   python3 embodied_ai_core.py
   ```

4. Run the affordance perception system:
   ```bash
   python3 affordance_perception.py
   ```

5. Run the situated learning demonstration:
   ```bash
   python3 situated_learning_demo.py
   ```

## Key Concepts Demonstrated

- **Embodied Cognition**: Intelligence that emerges from body-environment interaction
- **Affordance Perception**: Detection of action possibilities in the environment
- **Sensorimotor Learning**: Learning through physical interaction and experience
- **Situated Learning**: Context-dependent learning that adapts to environmental conditions
- **Morphological Computation**: Leveraging body properties for intelligent behavior
- **Ecological Psychology**: Direct perception-action coupling principles

## Expected Behavior

The embodied AI system will:
- Process sensory input to detect environmental affordances
- Use body schema to determine action feasibility
- Learn from sensorimotor experiences to improve performance
- Adapt behavior based on environmental context
- Execute actions that are appropriate to the current situation
- Provide natural language responses about its understanding