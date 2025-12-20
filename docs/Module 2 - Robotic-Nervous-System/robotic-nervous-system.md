---
sidebar_position: 1
---

# Module 1: The Robotic Nervous System (ROS 2)

## Overview

Welcome to the foundational module of your robotics journey! In this module, you'll explore the "nervous system" of modern robots - ROS 2 (Robot Operating System 2). Just as our nervous system enables communication between different parts of our body, ROS 2 enables seamless communication between different components of a robot.

This module will introduce you to the core concepts that power countless robots around the world, from autonomous vehicles to warehouse automation systems. By the end of this module, you'll understand how to design, build, and communicate with robotic systems using industry-standard tools.

## Learning Objectives

By the end of this module, you will be able to:

1. **Explain** the fundamental components of ROS 2 architecture (Nodes, Topics, Services)
2. **Implement** basic ROS 2 programs using the rclpy Python client library
3. **Create** and interpret URDF (Unified Robot Description Format) files for robot modeling
4. **Design** simple robot communication patterns using publishers and subscribers
5. **Troubleshoot** common ROS 2 communication issues

## Prerequisites

Before starting this module, you should have:

- Basic Python programming knowledge (variables, functions, classes)
- Familiarity with command-line interfaces
- Understanding of basic algebra and coordinate systems
- Access to a computer capable of running ROS 2 (Ubuntu 20.04/22.04, Windows 10/11, or macOS)

## Core Concepts

### 1. ROS 2 Nodes

Think of a **Node** as a process that performs computation. In robotics, nodes are individual programs that handle specific tasks like sensor processing, motion control, or decision making.

**Key characteristics:**
- Each node runs independently
- Nodes communicate with each other through topics and services
- Multiple nodes can run on the same machine or across a network

### 2. Topics and Message Passing

**Topics** enable asynchronous communication between nodes using a publish-subscribe pattern. Publishers send data to a topic, and subscribers receive that data.

**Example use cases:**
- Sensor data broadcasting (camera feeds, LIDAR scans)
- Robot state updates (position, battery level)
- Command distribution (velocity commands, goal positions)

### 3. Services

**Services** enable synchronous request-response communication. A client sends a request and waits for a response from a server.

**Example use cases:**
- Map saving/retrieval
- Navigation goal setting
- Robot calibration procedures

### 4. rclpy

**rclpy** is the Python client library for ROS 2. It provides the interface between your Python code and the ROS 2 middleware, allowing you to create nodes, publish/subscribe to topics, and provide/use services.

### 5. URDF (Unified Robot Description Format)

**URDF** is an XML-based format used to describe robot models. It defines the physical and visual properties of a robot including:
- Kinematic structure (joint connections)
- Physical properties (mass, inertia)
- Visual appearance (meshes, colors)
- Sensor placements

## Chapter Outline

This module is organized into the following sections:

1. [ROS 2 Installation and Setup](./installation.md)
2. [Understanding ROS 2 Architecture](./architecture.md)
3. [Creating Your First ROS 2 Node](./first-node.md)
4. [Topics and Publishers/Subscribers](./topics.md)
5. [Services and Clients](./services.md)
6. [Introduction to URDF](./urdf-intro.md)
7. [Mini-Projects and Exercises](./exercises.md)

## Measurable Outcomes

After completing this module, you will achieve these measurable outcomes:

- **Communication**: Successfully implement a publisher-subscriber pair that exchanges messages at 10Hz for 30 seconds
- **Service Implementation**: Create a service that accepts a mathematical operation request and returns the result
- **URDF Creation**: Design a URDF file for a simple 3-link robot arm with proper joint definitions
- **System Integration**: Build a multi-node system that coordinates to move a simulated robot in a square pattern

## Getting Started

Ready to dive into the world of robotics? Start with the [Installation Guide](./installation.md) to set up your development environment, then proceed through each chapter to build your understanding progressively.

Each chapter includes hands-on exercises designed to reinforce the concepts through practical implementation. Don't just read - code along and experiment with the examples to solidify your understanding.

---

## Next Steps

Continue to the [Installation and Setup](./installation.md) chapter to prepare your environment for ROS 2 development.