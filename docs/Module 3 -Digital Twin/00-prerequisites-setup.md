---
sidebar_position: 0
---

# Prerequisites and Setup Guide

## Overview

Before diving into digital twin implementation with Gazebo and Unity, you need to set up your development environment with the required software and knowledge. This guide will walk you through all the prerequisites needed to successfully complete the modules in this course.

## Technical Requirements

### Operating System
- **Recommended**: Ubuntu 22.04 LTS
- **Alternative**: Windows 10/11 with WSL2 (Windows Subsystem for Linux)
- **MacOS**: Supported but may require additional configuration

### Hardware Requirements
- **CPU**: Multi-core processor (Intel i5 or equivalent)
- **RAM**: 8GB minimum, 16GB recommended
- **GPU**: Dedicated graphics card with OpenGL 3.3+ support (required for Unity)
- **Storage**: 10GB free space for complete installation
- **Network**: Stable internet connection for downloads and updates

## Software Installation

### ROS 2 Installation (Humble Hawksbill)

For Ubuntu users:

```bash
# Setup locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install python3-colcon-common-extensions

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### Gazebo Installation

Install Gazebo Garden:

```bash
curl -sSL http://get.gazebosim.org | sh

# Verify installation
gz --version
```

### Unity Installation

1. Download Unity Hub from https://unity.com/download
2. Install Unity Hub and create an account
3. Install Unity 2021.3 LTS through Unity Hub
4. Install additional modules if needed for Linux development

## Knowledge Prerequisites

### Essential Knowledge Areas

Before starting this course, you should have:

#### ROS Basics
- Understanding of ROS 2 concepts (nodes, topics, services, actions)
- Experience with ROS 2 tools (ros2 run, ros2 launch, etc.)
- Knowledge of ROS 2 workspace creation and compilation

#### Python Programming
- Basic to intermediate Python knowledge
- Understanding of object-oriented programming concepts
- Experience with Python libraries for robotics

#### Linux Command Line
- Comfortable with bash commands and file management
- Understanding of package management (apt/yum)
- Experience with text editors (nano, vim, or similar)

#### 3D Concepts
- Basic understanding of coordinate systems (world, local, camera)
- Knowledge of transformations (rotation, translation, scaling)
- Understanding of basic geometric concepts

#### Physics Fundamentals
- Knowledge of kinematics (position, velocity, acceleration)
- Understanding of dynamics (forces, torques, mass)
- Basic grasp of friction and collision concepts

## Recommended Learning Resources

### ROS 2 Resources
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- ROS 2 tutorials and examples
- "Programming Robots with ROS" by Morgan Quigley

### Python Resources
- Python official documentation
- NumPy and SciPy tutorials for scientific computing
- Object-oriented programming concepts

### 3D and Graphics
- Basic 3D math concepts
- Coordinate system transformations
- OpenGL or graphics programming basics

## Setup Verification

### Verify ROS 2 Installation

```bash
# Check ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 topic list

# Test basic functionality
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```

### Verify Gazebo Installation

```bash
# Launch Gazebo
gz sim
# Should open Gazebo GUI with default environment
```

### Verify Unity Installation

1. Launch Unity Hub
2. Create a new 3D project
3. Verify that the editor opens without errors

## Troubleshooting Common Issues

### ROS 2 Issues
- **Problem**: ROS 2 commands not found
  - **Solution**: Ensure ROS 2 environment is sourced: `source /opt/ros/humble/setup.bash`

- **Problem**: Permission errors with Gazebo
  - **Solution**: Check user permissions and graphics drivers

### Unity Issues
- **Problem**: Unity crashes on startup
  - **Solution**: Update graphics drivers, try running with `-force-opengl` flag

- **Problem**: Performance issues in Unity
  - **Solution**: Check hardware requirements, reduce rendering quality

### Network Issues (Unity-ROS Connection)
- **Problem**: Cannot connect Unity to ROS
  - **Solution**: Check firewall settings, verify IP addresses, ensure ROS bridge is running

## Optional Enhancements

### Advanced Tools (Optional)
- **RViz2**: For ROS visualization
- **Gazebo Web Interface**: Browser-based simulation monitoring
- **Unity ML-Agents**: For reinforcement learning with robots
- **Docker**: For containerized development environments

### Additional Sensors (Optional)
- Force/Torque sensors
- GPS simulation
- Thermal cameras
- Multi-cameras for stereo vision

## Getting Started Checklist

Before proceeding with the course, ensure you have completed:

- [ ] Installed ROS 2 Humble Hawksbill
- [ ] Installed Gazebo Garden
- [ ] Installed Unity 2021.3 LTS
- [ ] Verified all installations work
- [ ] Installed Unity Robotics Package
- [ ] Installed ROS-TCP-Connector
- [ ] Tested basic ROS functionality
- [ ] Run a simple Gazebo simulation
- [ ] Created a basic Unity scene
- [ ] Connected Unity to ROS (optional for advanced users)

## Next Steps

Once you have completed all prerequisites:

1. Start with the Introduction to Digital Twins chapter
2. Progress through each module sequentially
3. Complete the hands-on exercises in each chapter
4. Build your own digital twin system in the final module

Remember that setting up the development environment properly is crucial for success in this course. Take time to ensure each component is working correctly before moving forward.