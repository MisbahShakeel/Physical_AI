[Diagram: Physical AI Architecture - ROS 2 Communication Patterns for Humanoid Robot Control]

This diagram illustrates the fundamental communication patterns in ROS 2 for humanoid robot control systems. It demonstrates how nodes communicate through topics (publish/subscribe), services (request/response), and actions (goal-based communication). The visualization includes:

- **Topics**: Asynchronous communication for continuous data streams like sensor readings and actuator commands
- **Services**: Synchronous request/response communication for discrete operations like calibration or configuration
- **Actions**: Goal-based communication for long-running tasks with feedback, such as navigation or manipulation

The diagram shows how these communication patterns work together to create a robust, distributed control architecture for humanoid robots, enabling real-time coordination between perception, planning, and control nodes.