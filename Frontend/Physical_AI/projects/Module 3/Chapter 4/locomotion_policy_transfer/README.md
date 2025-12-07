# Locomotion Policy Transfer Project

This project demonstrates the complete pipeline for training a humanoid locomotion policy in Isaac Sim and transferring it to a ROS 2 controlled robot.

## Project Structure

- `isaac_sim_config/` - Isaac Sim configuration files for the humanoid robot
- `ros2_control/` - ROS 2 control scripts for the real robot
- `policy_files/` - Trained policy files and models
- `transfer_scripts/` - Scripts for policy transfer and adaptation
- `evaluation/` - Scripts for evaluating policy performance

## Components

### Isaac Sim Configuration
- Robot model definition for simulation
- Environment setup for locomotion training
- Domain randomization parameters

### ROS 2 Control Scripts
- Joint command interfaces
- State monitoring and safety checks
- Policy execution wrapper

### Policy Files
- Trained neural network models
- Policy configuration files
- Performance metrics and logs

### Transfer Scripts
- Action space remapping utilities
- Domain adaptation mechanisms
- Safety validation tools

## Running the Project

1. Train a locomotion policy in Isaac Sim using the provided environment
2. Export the trained policy
3. Use the transfer scripts to adapt the policy for the real robot
4. Deploy the adapted policy to the ROS 2 controlled robot
5. Monitor performance and adjust as needed

## Key Concepts Demonstrated

- Sim-to-real transfer for humanoid locomotion
- Domain randomization for robust policy training
- Safety mechanisms for real-world deployment
- Action space adaptation between simulation and reality
- Performance evaluation across domains