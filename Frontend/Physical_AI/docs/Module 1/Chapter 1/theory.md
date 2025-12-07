---
title: "ROS 2 Nodes, Topics, and Services"
---

## Learning Objectives

By the end of this chapter, you will be able to:
- Define and explain the fundamental concepts of ROS 2 nodes, topics, and services
- Create and run a basic ROS 2 node using Python
- Understand the publish-subscribe communication model in ROS 2
- Implement simple publisher and subscriber nodes that communicate over topics
- Understand the request-response communication model using services
- Differentiate between topics and services, and know when to use each
- Configure and run ROS 2 nodes with proper package structure

## Core Theory

### Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

ROS 2 is designed to be the successor to ROS 1, addressing many of the limitations of the original system, particularly in the areas of:
- Real-time support
- Multi-robot systems
- Security
- Deterministic behavior
- Professional deployment scenarios

### ROS 2 Architecture

ROS 2 uses a distributed architecture where processes (called "nodes") can be run on the same system or across multiple systems. Communication between these nodes happens through a publish-subscribe model using topics, request-response using services, or via actions for more complex, long-running tasks.

The communication layer in ROS 2 is built on top of DDS (Data Distribution Service), which provides a standardized middleware for real-time, scalable, and reliable data exchange.

### Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are designed to perform specific, narrowly-focused functions. Multiple nodes are usually combined together to perform complex robot behavior.

Key characteristics of nodes:
- Each node is a process that performs computation
- Nodes can publish or subscribe to topics
- Nodes can provide or call services
- Nodes can have parameters
- Nodes are managed by a node lifecycle (creation, configuration, activation, deactivation, cleanup, destruction)

### Topics and Publish-Subscribe Model

Topics are named buses over which nodes exchange messages. The communication is based on a publish-subscribe pattern where:
- Publishers send messages to a topic
- Subscribers receive messages from a topic
- Multiple publishers can publish to the same topic
- Multiple subscribers can subscribe to the same topic
- Communication is asynchronous
- Publishers and subscribers are decoupled in time and space

The publish-subscribe model is ideal for streaming data where:
- The publisher doesn't need to know who receives the data
- The subscriber doesn't need to know where the data comes from
- Real-time delivery is important
- Multiple subscribers can process the same data

### Services and Request-Response Model

Services provide a request-response communication pattern where:
- A client sends a request to a service
- The service processes the request and sends back a response
- Communication is synchronous
- The client waits for the response

Services are ideal for:
- Actions that need to return a result
- Requesting specific information
- Operations that need confirmation of completion
- Synchronous communication where timing matters

### Message Types

ROS 2 uses message types to define the structure of data exchanged between nodes. Messages are defined in .msg files for topics and .srv files for services. These definitions are used to generate code in various programming languages.

Common message types include:
- Standard types (std_msgs): basic data types like integers, floats, strings
- Geometry types (geometry_msgs): poses, points, vectors
- Sensor types (sensor_msgs): camera data, laser scans, IMU data
- Navigation types (nav_msgs): path planning and navigation data

### Quality of Service (QoS) Settings

ROS 2 provides Quality of Service (QoS) settings that allow fine-tuning of communication behavior:
- Reliability: best effort or reliable delivery
- Durability: volatile or transient local
- History: keep all or keep last N messages
- Deadline and lifespan settings for time-sensitive communication

### ROS 2 Client Libraries

ROS 2 provides client libraries for different programming languages:
- rclcpp: C++ client library
- rclpy: Python client library (the focus of this textbook)
- Other languages like Rust, Java, and C# are also supported

### Launch Files and Management

ROS 2 uses launch files to start multiple nodes at once and manage their lifecycle. Launch files can be written in Python and provide:
- Node startup configuration
- Parameter setting
- Remapping of topics and services
- Conditional execution
- Process management

This foundational understanding of nodes, topics, and services forms the basis for all ROS 2 communication and is essential for developing more complex robotic systems.