---
id: chapter1
title: ROS 2 Fundamentals
sidebar_position: 1
---

# Chapter 1: ROS 2 Graph Basics - Nodes, Topics, Services

Welcome to Module 1: The Robotic Nervous System (ROS 2)! This chapter introduces the fundamental concepts of the ROS 2 graph, which forms the backbone of communication in a ROS 2 system. Understanding these basics is crucial for developing any robotics application with ROS 2.

## What is the ROS 2 Graph?

The ROS 2 graph is a network of interconnected **nodes** that communicate with each other using various mechanisms like **topics** and **services**. It's a distributed system, meaning different parts of your robot's software can run as separate processes, potentially on different machines, and still interact seamlessly.

## Nodes: The Workers of the Graph

In ROS 2, a **node** is essentially an executable process that performs a specific task. Think of a node as a single, focused program. For example:
- A camera driver node that publishes image data.
- A motor control node that subscribes to movement commands.
- A navigation node that processes sensor data and publishes velocity commands.

Nodes are designed to be modular and reusable. Each node should ideally have a single responsibility.

## Topics: The Communication Highways

**Topics** are the primary mechanism for asynchronous, many-to-many communication in ROS 2. Nodes publish messages to topics, and other nodes subscribe to those topics to receive the messages.

-   **Publisher**: A node that sends data to a topic.
-   **Subscriber**: A a node that receives data from a topic.
-   **Message**: The data format sent over a topic. Messages have a well-defined structure.

This communication is unidirectional: data flows from publishers to subscribers. A single topic can have multiple publishers and multiple subscribers.

**Example**: A "temperature_sensor" node could publish temperature readings to a `/robot/temperature` topic. A "display" node could subscribe to this topic to show the temperature, and a "safety_monitor" node could also subscribe to trigger an alarm if the temperature exceeds a certain threshold.

## Services: Request-Response Interactions

**Services** provide a synchronous, one-to-one request-response communication mechanism. Unlike topics, where data flows continuously, services are used when a node needs to request a computation or an action from another node and wait for a response.

-   **Service Server**: A node that offers a service and processes requests.
-   **Service Client**: A node that sends a request to a service server and waits for a response.

**Example**: A "robot_arm_control" node could offer a service called `/set_arm_position`. A "task_planner" node could act as a client, requesting the arm to move to a specific position and waiting for confirmation that the movement is complete.

## Summary

Nodes are the fundamental processing units. Topics enable flexible, one-way data streaming, perfect for continuous data like sensor readings. Services provide explicit request-response interactions for actions and computations. Together, they form the dynamic and powerful ROS 2 graph, allowing complex robotic systems to be built from modular components.

---

**Next**: In Chapter 2, we will dive into implementing Python agents using `rclpy` to build and interact with the ROS 2 graph.
