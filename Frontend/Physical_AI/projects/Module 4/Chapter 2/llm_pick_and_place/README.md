# LLM Pick and Place Project

This project implements a system where an LLM plans a multi-step pick and place task for a simulated humanoid robot, demonstrating cognitive planning and action sequencing.

## Project Structure

- `llm_pick_place_controller.py` - Main controller that integrates LLM planning with robot execution
- `task_planner.py` - LLM-based task decomposition and planning
- `action_executor.py` - ROS 2 action execution for pick and place operations
- `environment_simulator.py` - Simulated environment with objects and locations
- `config.yaml` - Configuration parameters for the LLM pick and place system

## Components

### LLM Task Planning
- Natural language command interpretation
- Task decomposition into pick and place subtasks
- Object and location recognition
- Action sequence generation
- Safety and feasibility validation

### Action Execution
- Navigation to object location
- Object detection and grasping
- Transport to destination
- Object placement
- Execution monitoring and error handling

### Integration with Navigation
- Path planning to object locations
- Obstacle avoidance during transport
- Precise positioning for manipulation
- Coordination between navigation and manipulation

## Running the Project

1. Set up your OpenAI API key as an environment variable:
   ```bash
   export OPENAI_API_KEY='your-api-key-here'
   ```

2. Run the main controller:
   ```bash
   python llm_pick_place_controller.py
   ```

3. Send commands via ROS 2 topics, such as:
   - "Pick up the red cube and place it on the table"
   - "Move the book from shelf to desk"

## Key Concepts Demonstrated

- LLM-based cognitive planning for robotics
- Integration of high-level planning with low-level execution
- Natural language to action sequence conversion
- Safety validation in LLM-driven systems
- Coordination between navigation and manipulation