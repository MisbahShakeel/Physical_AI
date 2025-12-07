# LiDAR Navigation Simulation

This project demonstrates a humanoid robot using LiDAR data to navigate and avoid obstacles in a simulated environment.

## Components

- `lidar_navigation_agent.py`: Python agent that processes LiDAR data and controls robot navigation
- `simple_room.world`: Gazebo world with obstacles for navigation testing
- `lidar_navigation.launch`: Launch file to start the simulation

## How it Works

The navigation agent implements a simple reactive behavior:

1. **Front Detection**: Monitors LiDAR readings in front of the robot (±30 degrees)
2. **Obstacle Avoidance**: If obstacles are detected within safe distance (1m), the robot turns away
3. **Side Detection**: Uses left/right side readings to determine turning direction
4. **Forward Movement**: Moves forward when clear path is detected

## Running the Simulation

1. Make sure you have ROS 2 installed with Gazebo and necessary dependencies
2. Launch the simulation:
   ```bash
   # Launch Gazebo with the world and robot
   roslaunch your_package lidar_navigation.launch
   ```

3. The robot will start navigating the environment, avoiding obstacles using LiDAR data

## Configuration Parameters

- `linear_speed`: Forward speed of the robot (default: 0.5 m/s)
- `angular_speed`: Turning speed of the robot (default: 0.8 rad/s)
- `safe_distance`: Minimum safe distance to obstacles (default: 1.0 m)
- `front_angle_range`: Angular range for front obstacle detection (default: 30 degrees)

## Expected Behavior

The humanoid robot equipped with a LiDAR sensor will:
- Move forward when no obstacles are detected in front
- Turn away from obstacles when they are too close
- Navigate around obstacles to continue moving
- Stop safely if no clear path is available