---
sidebar_position: 2
---

# Installation and Setup

## Overview

Setting up your ROS 2 development environment is the first crucial step in your robotics journey. This guide will walk you through the installation process for different operating systems and ensure you have everything needed to start building robotic applications.

## System Requirements

### Minimum Requirements
- **CPU**: Multi-core processor (Intel i5 or equivalent)
- **RAM**: 8 GB (16 GB recommended)
- **Storage**: 20 GB free space
- **OS**: Ubuntu 20.04/22.04 LTS, Windows 10/11, or macOS 12+

### Recommended Specifications
- **CPU**: Intel i7 or AMD Ryzen equivalent
- **RAM**: 16 GB or more
- **GPU**: NVIDIA GPU with CUDA support (for advanced perception tasks)

## Ubuntu Installation (Recommended)

### Step 1: Set Locale
```bash
locale  # Check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 2: Add ROS 2 Repository
```bash
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

### Step 4: Install colcon and other tools
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update
```

### Step 5: Source the Environment
Add to your `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Windows Installation

### Step 1: Install Prerequisites
1. Install [Chocolatey](https://chocolatey.org/install)
2. Install Python 3.8 or higher from [python.org](https://www.python.org/)
3. Install Git for Windows

### Step 2: Install ROS 2 via Binary
1. Download the ROS 2 Humble Hawksbill Windows installer
2. Run the installer and follow the prompts
3. Add ROS 2 to your PATH environment variable

### Step 3: Install Additional Tools
```cmd
pip install -U colcon-common-extensions
pip install -U rosdep
```

## macOS Installation

### Step 1: Install Homebrew
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

### Step 2: Install ROS 2 via Binary
```bash
brew install ros/humble/ros-humble-desktop
```

### Step 3: Source the Environment
Add to your `~/.zshrc` or `~/.bash_profile`:
```bash
source /opt/ros/humble/setup.bash
```

## Verification Test

After installation, verify your setup with these commands:

```bash
# Check ROS 2 installation
ros2 --version

# Test basic functionality
ros2 run demo_nodes_cpp talker
```

In another terminal:
```bash
ros2 run demo_nodes_py listener
```

You should see messages passing between the talker and listener nodes.

## Common Installation Issues and Solutions

### Issue: Permission denied for rosdep init
**Solution:**
```bash
sudo rosdep init
rosdep update
```

### Issue: Python package conflicts
**Solution:** Use a virtual environment:
```bash
python3 -m venv ros2_env
source ros2_env/bin/activate  # On Windows: ros2_env\Scripts\activate
pip install -U colcon-common-extensions
```

### Issue: Missing dependencies
**Solution:** Update package lists and install dependencies:
```bash
sudo apt update
sudo apt install build-essential cmake git python3-colcon-common-extensions python3-rosdep python3-vcstool
```

## Setting Up Your Workspace

Create your first ROS 2 workspace:

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Development Environment Setup

### Recommended IDEs
1. **Visual Studio Code** with ROS extension
2. **PyCharm** with Python ROS plugins
3. **CLion** for C++ development

### Essential Tools
- **RViz2**: 3D visualization tool
- **rqt**: GUI tools for ROS 2
- **ros2cli**: Command-line tools

Install additional tools:
```bash
sudo apt install ros-humble-rviz2 ros-humble-rqt ros-humble-ros2cli
```

## Troubleshooting

### If ROS 2 commands are not found:
1. Check if the environment is sourced: `echo $ROS_DISTRO`
2. If empty, source the setup file: `source /opt/ros/humble/setup.bash`

### If you encounter encoding errors:
Set your locale properly:
```bash
export LC_ALL=C.UTF-8
export LANG=C.UTF-8
```

## Next Steps

Once you've successfully installed and verified your ROS 2 environment, continue to the [Understanding ROS 2 Architecture](./architecture.md) chapter to learn about the fundamental concepts that make ROS 2 powerful for robotics development.