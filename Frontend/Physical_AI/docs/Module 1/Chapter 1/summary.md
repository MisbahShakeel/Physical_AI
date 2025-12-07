---
title: Key Takeaways and Review Questions - ROS 2 Nodes, Topics, and Services
---

## Key Takeaways

- **Nodes** are executable processes that perform specific functions within the ROS 2 framework and communicate with other nodes.
- **Topics** enable asynchronous communication through a publish-subscribe model, ideal for streaming data where multiple publishers and subscribers can interact.
- **Services** provide synchronous request-response communication, suitable for operations requiring a specific result or confirmation.
- **Messages** have standardized types that define the structure of data exchanged between nodes, with different message types for various data needs.
- **Quality of Service (QoS)** settings allow fine-tuning of communication behavior to meet specific timing and reliability requirements.
- The **rclpy** library provides Python bindings for ROS 2, enabling Python-based node development.
- **Launch files** written in Python allow for starting multiple nodes with configuration parameters and lifecycle management.

## Review Questions

1. **Basic Understanding**: What is the primary difference between ROS 2 topics and services in terms of communication pattern?

2. **Application**: Describe a scenario where you would use a topic instead of a service for communication between two ROS 2 nodes. Justify your choice.

3. **Conceptual**: Explain the publish-subscribe communication model in ROS 2. What are the advantages of this approach for robot systems?

4. **Technical**: What is the role of Quality of Service (QoS) settings in ROS 2 communication? Provide examples of when you might use reliable vs. best-effort delivery.

5. **Analysis**: Compare and contrast the use of nodes, topics, and services in terms of:
   - Synchronization (synchronous vs. asynchronous)
   - Number of participants (one-to-one vs. one-to-many vs. many-to-many)
   - Data flow direction (bidirectional vs. unidirectional)

6. **Implementation**: If you were to design a simple robot system with a sensor node and an actuator node, how would you structure the communication using ROS 2 concepts? What would be the role of each node and what type of communication (topic/service) would you use?

7. **Advanced**: How does the DDS (Data Distribution Service) middleware underlying ROS 2 influence the design of the publish-subscribe and request-response communication patterns?

8. **Critical Thinking**: What are the potential challenges of using only topics for communication in a complex robot system? How do services address some of these challenges?

9. **Practical Application**: When would you choose to use a service instead of a topic for requesting the current position of a robot? What are the trade-offs?

10. **Synthesis**: Design a simple ROS 2 system architecture for a mobile robot that needs to navigate a room. Identify at least three nodes and specify what type of communication (topic or service) you would use between them, justifying your choices.