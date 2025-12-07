## RAG Knowledge Chunks - Chapter 1: ROS 2 Nodes, Topics, and Services

### Core Definition: ROS 2
ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. ROS 2 addresses limitations of ROS 1, particularly in real-time support, multi-robot systems, security, and professional deployment scenarios.

### Core Definition: Node
A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are designed to perform specific, narrowly-focused functions. Multiple nodes are usually combined together to perform complex robot behavior. Key characteristics include: each node is a process that performs computation, nodes can publish or subscribe to topics, nodes can provide or call services, nodes can have parameters, and nodes are managed by a node lifecycle.

### Core Definition: Topic
Topics are named buses over which nodes exchange messages. The communication is based on a publish-subscribe pattern where publishers send messages to a topic and subscribers receive messages from a topic. Multiple publishers can publish to the same topic, multiple subscribers can subscribe to the same topic, communication is asynchronous, and publishers and subscribers are decoupled in time and space.

### Core Definition: Service
Services provide a request-response communication pattern where a client sends a request to a service and the service processes the request and sends back a response. Communication is synchronous and the client waits for the response. Services are ideal for actions that need to return a result, requesting specific information, operations that need confirmation of completion, and synchronous communication where timing matters.

### Communication Pattern: Publish-Subscribe Model
The publish-subscribe model in ROS 2 is ideal for streaming data where the publisher doesn't need to know who receives the data, the subscriber doesn't need to know where the data comes from, real-time delivery is important, and multiple subscribers can process the same data. This model enables asynchronous, decoupled communication between nodes.

### Communication Pattern: Request-Response Model
The request-response model in ROS 2 provides synchronous communication where a client sends a request and waits for a response from a service. This pattern is suitable for operations requiring specific results or confirmations, making it different from the asynchronous nature of topics.

### Message Types in ROS 2
ROS 2 uses message types to define the structure of data exchanged between nodes. Messages are defined in .msg files for topics and .srv files for services. These definitions are used to generate code in various programming languages. Common message types include: standard types (std_msgs) for basic data types like integers, floats, strings; geometry types (geometry_msgs) for poses, points, vectors; sensor types (sensor_msgs) for camera data, laser scans, IMU data; and navigation types (nav_msgs) for path planning and navigation data.

### Quality of Service (QoS) Settings
Quality of Service (QoS) settings in ROS 2 allow fine-tuning of communication behavior with options for: reliability (best effort or reliable delivery), durability (volatile or transient local), history (keep all or keep last N messages), and deadline and lifespan settings for time-sensitive communication.

### Client Libraries in ROS 2
ROS 2 provides client libraries for different programming languages including: rclcpp for C++, rclpy for Python, and other languages like Rust, Java, and C#. The rclpy library specifically provides Python bindings for ROS 2, enabling Python-based node development.

### Launch Files in ROS 2
ROS 2 uses launch files to start multiple nodes at once and manage their lifecycle. Launch files can be written in Python and provide node startup configuration, parameter setting, remapping of topics and services, conditional execution, and process management.

### When to Use Topics vs Services
Use topics for streaming data, asynchronous communication, and when multiple subscribers need the same data. Use services for operations requiring a specific result, requesting specific information, operations needing confirmation of completion, and synchronous communication where timing matters.

### Key Takeaway: ROS 2 Architecture
ROS 2 uses a distributed architecture where processes (called "nodes") can be run on the same system or across multiple systems. Communication between these nodes happens through a publish-subscribe model using topics, request-response using services, or via actions for more complex, long-running tasks. The communication layer is built on top of DDS (Data Distribution Service).