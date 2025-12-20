---
sidebar_position: 7
---

# Introduction to URDF

## Overview

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. Understanding URDF is crucial for robot simulation, visualization, and control. In this chapter, you'll learn to create, understand, and work with robot descriptions that define everything from physical properties to joint relationships.

## What is URDF?

URDF stands for **Unified Robot Description Format**. It's an XML-based format that describes:

- **Physical structure**: Links and their properties
- **Kinematic structure**: Joint relationships and constraints
- **Visual appearance**: How the robot looks in simulation
- **Collision properties**: Physics interactions
- **Inertial properties**: Mass, center of mass, moments of inertia

## URDF Structure

A basic URDF file follows this structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links define rigid bodies -->
  <link name="link_name">
    <inertial>...</inertial>
    <visual>...</visual>
    <collision>...</collision>
  </link>

  <!-- Joints define connections between links -->
  <joint name="joint_name" type="joint_type">
    <parent link="parent_link"/>
    <child link="child_link"/>
    <origin xyz="x y z" rpy="roll pitch yaw"/>
  </joint>
</robot>
```

## Links: The Building Blocks

### Basic Link Structure

```xml
<link name="base_link">
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>

  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.5 0.5 0.5"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.5 0.5 0.5"/>
    </geometry>
  </collision>
</link>
```

### Link Components

1. **Inertial**: Physical properties for physics simulation
2. **Visual**: How the link appears in visualization
3. **Collision**: How the link interacts with physics

## Joint Types

### 1. Fixed Joint
```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>
```
Fixed joints create rigid connections with no degrees of freedom.

### 2. Revolute Joint
```xml
<joint name="revolute_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```
Revolute joints rotate around a single axis with limits.

### 3. Continuous Joint
```xml
<joint name="continuous_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0.2 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```
Continuous joints can rotate infinitely around an axis.

### 4. Prismatic Joint
```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="slider_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="10" velocity="0.5"/>
</joint>
```
Prismatic joints move linearly along an axis.

## Visual and Collision Elements

### Geometry Types

1. **Box**
```xml
<geometry>
  <box size="0.1 0.2 0.3"/>
</geometry>
```

2. **Cylinder**
```xml
<geometry>
  <cylinder radius="0.1" length="0.2"/>
</geometry>
```

3. **Sphere**
```xml
<geometry>
  <sphere radius="0.1"/>
</geometry>
```

4. **Mesh**
```xml
<geometry>
  <mesh filename="package://robot_description/meshes/link.dae" scale="1 1 1"/>
</geometry>
```

### Materials
```xml
<material name="red">
  <color rgba="1 0 0 1"/>
</material>

<material name="blue">
  <color rgba="0 0 1 1"/>
</material>

<material name="yellow">
  <color rgba="1 1 0 1"/>
</material>
```

## Complete Robot Example

Let's create a simple 2-link manipulator:

```xml
<?xml version="1.0"?>
<robot name="simple_manipulator">

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.2"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- First arm link -->
  <link name="arm_link_1">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.25"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint connecting base to first arm -->
  <joint name="base_to_arm1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link_1"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- End effector link -->
  <link name="end_effector">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint connecting first arm to end effector -->
  <joint name="arm1_to_ee" type="fixed">
    <parent link="arm_link_1"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>

</robot>
```

## Xacro: XML Macros for URDF

Xacro is a macro language that makes URDF more readable and maintainable:

### Basic Xacro Example

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="base_mass" value="1.0" />
  <xacro:property name="arm_length" value="0.5" />

  <!-- Define a macro for creating links -->
  <xacro:macro name="simple_link" params="name mass length color">
    <link name="${name}">
      <inertial>
        <mass value="${mass}"/>
        <origin xyz="0 0 ${length/2}"/>
        <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
      </inertial>

      <visual>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.05" length="${length}"/>
        </geometry>
        <material name="${name}_material">
          <color rgba="${color}"/>
        </material>
      </visual>

      <collision>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.05" length="${length}"/>
        </geometry>
      </collision>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:simple_link name="base_link" mass="1.0" length="0.2" color="0.5 0.5 0.5 1"/>

</robot>
```

## URDF Validation and Tools

### Validating URDF Files

```bash
# Check if URDF is well-formed
check_urdf /path/to/robot.urdf

# Get robot information
ros2 run urdf_parser_kinematics_tutorials display_urdf /path/to/robot.urdf
```

### Visualizing URDF

```bash
# Launch RViz with robot model
ros2 run rviz2 rviz2

# Use robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat robot.urdf)
```

## Practical Example: TurtleBot3 URDF

Let's examine a real-world example by creating a simplified TurtleBot3:

```xml
<?xml version="1.0"?>
<robot name="turtlebot3_burger" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.033" />
  <xacro:property name="wheel_separation" value="0.160" />
  <xacro:property name="wheel_base" value="0.144" />

  <!-- Base link -->
  <link name="base_footprint">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.001 0.001 0.001"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.018" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <origin xyz="-0.032 0 0.021" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/bases/burger_base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="light_black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="-0.032 0 0.021" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/bases/burger_base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="-0.016 0 0.017" rpy="0 0 0"/>
      <mass value="8.2573504e-01"/>
      <inertia ixx="2.2124416e-03" ixy="0" ixz="0" iyy="2.1119994e-03" iyz="0" izz="2.0000000e-03"/>
    </inertial>
  </link>

  <!-- Wheels -->
  <xacro:macro name="wheel" params="prefix x_reflect y_reflect">
    <link name="${prefix}_wheel_link">
      <visual>
        <origin xyz="0 0 0" rpy="1.57 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="0.018"/>
        </geometry>
        <material name="dark">
          <color rgba="0.2 0.2 0.2 1.0"/>
        </material>
      </visual>

      <collision>
        <origin xyz="0 0 0" rpy="1.57 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="0.018"/>
        </geometry>
      </collision>

      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="2.8498940e-02"/>
        <inertia ixx="1.1175580e-05" ixy="0" ixz="0" iyy="1.1175580e-05" iyz="0" izz="2.1117930e-05"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel_link"/>
      <origin xyz="0 ${y_reflect*wheel_separation/2} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <!-- Create wheels -->
  <xacro:wheel prefix="left" x_reflect="1" y_reflect="1"/>
  <xacro:wheel prefix="right" x_reflect="1" y_reflect="-1"/>

  <!-- Castor wheel -->
  <link name="caster_wheel_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${wheel_radius/2}"/>
      </geometry>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${wheel_radius/2}"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.01"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel_link"/>
    <origin xyz="-0.081 0 -0.004" rpy="0 0 0"/>
  </joint>

</robot>
```

## Working with URDF in ROS 2

### Robot State Publisher

The robot state publisher is essential for visualizing and working with URDF:

```python
#!/usr/bin/env python3
"""
URDF Robot State Publisher Example

This node publishes robot state based on URDF description.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class RobotStatePublisher(Node):
    """Publish robot state based on joint positions."""

    def __init__(self):
        super().__init__('robot_state_publisher')

        # Create joint state publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Create transform broadcaster for TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create timer to publish state
        self.timer = self.create_timer(0.1, self.publish_robot_state)

        # Simulated joint positions
        self.joint_positions = {
            'base_to_arm1': 0.0,
            'arm1_to_arm2': 0.0
        }

        self.get_logger().info('Robot State Publisher started')

    def publish_robot_state(self):
        """Publish joint states and transforms."""
        # Create joint state message
        msg = JointState()
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Update joint positions (simulated movement)
        self.joint_positions['base_to_arm1'] = math.sin(
            self.get_clock().now().nanoseconds / 1e9)
        self.joint_positions['arm1_to_arm2'] = math.cos(
            self.get_clock().now().nanoseconds / 1e9)

        # Publish joint states
        self.joint_pub.publish(msg)

        # Publish transforms
        self.publish_transforms(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## URDF Best Practices

### 1. Organize with Xacro
Use Xacro for complex robots to avoid duplication and improve maintainability.

### 2. Proper Inertial Values
Accurate inertial values are crucial for physics simulation:
- Mass should be realistic
- Center of mass should be correctly positioned
- Moments of inertia should be properly calculated

### 3. Separate Visual and Collision Models
Use different geometry for visual and collision:
- Visual: Detailed for appearance
- Collision: Simplified for performance

### 4. Consistent Naming
Use consistent naming conventions:
- Links: descriptive names (base_link, arm_link_1, gripper_left_finger)
- Joints: indicate parent-child relationship (base_to_arm, arm_to_gripper)

### 5. Use Proper Units
- Length: meters
- Mass: kilograms
- Angles: radians
- Time: seconds

## Debugging URDF

### Common Issues

1. **Invalid joint connections**: Check parent-child relationships
2. **Floating point precision**: Use appropriate precision for transformations
3. **Inertial values**: Check for negative or zero values
4. **File paths**: Ensure mesh files are accessible

### Debugging Tools

```bash
# Validate URDF
check_urdf your_robot.urdf

# View robot tree
ros2 run urdf_parser_kinematics_tutorials display_urdf your_robot.urdf

# Launch with visualization
ros2 launch your_package your_robot.launch.py
```

## Mini-Exercise: Create a Simple Robot URDF

Create a URDF file for a simple wheeled robot with:
1. A base link
2. Two wheels connected with revolute joints
3. One castor wheel with a fixed joint
4. Proper visual and collision properties
5. Use Xacro for cleaner code

<details>
<summary>Click here for the solution</summary>

```xml
<?xml version="1.0"?>
<robot name="simple_wheeled_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.05" />
  <xacro:property name="wheel_separation" value="0.4" />
  <xacro:property name="base_length" value="0.5" />
  <xacro:property name="base_width" value="0.3" />
  <xacro:property name="base_height" value="0.15" />

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="orange">
        <color rgba="1 0.5 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
        ixx="0.2" ixy="0" ixz="0"
        iyy="0.4" iyz="0"
        izz="0.5"/>
    </inertial>
  </link>

  <!-- Macro for wheels -->
  <xacro:macro name="wheel" params="prefix x_pos y_pos">
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="1.5708 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0.2 0.2 0.2 1"/>
        </material>
      </visual>

      <collision>
        <origin xyz="0 0 0" rpy="1.5708 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>

      <inertial>
        <mass value="0.5"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia
          ixx="0.005" ixy="0" ixz="0"
          iyy="0.005" iyz="0"
          izz="0.01"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x_pos} ${y_pos} 0" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Create wheels -->
  <xacro:wheel prefix="left" x_pos="0.1" y_pos="${wheel_separation/2}"/>
  <xacro:wheel prefix="right" x_pos="0.1" y_pos="${-wheel_separation/2}"/>

  <!-- Castor wheel -->
  <link name="castor_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${wheel_radius/2}"/>
      </geometry>
      <material name="dark_grey">
        <color rgba="0.3 0.3 0.3 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${wheel_radius/2}"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
        ixx="0.0001" ixy="0" ixz="0"
        iyy="0.0001" iyz="0"
        izz="0.0001"/>
    </inertial>
  </link>

  <joint name="castor_joint" type="fixed">
    <parent link="base_link"/>
    <child link="castor_wheel"/>
    <origin xyz="-0.2 0 -${wheel_radius/2}" rpy="0 0 0"/>
  </joint>

</robot>
```

</details>

## Summary

In this chapter, you've learned:
- What URDF is and why it's important for robotics
- The structure of URDF files and their components
- How to create links and joints for robot models
- How to use Xacro for more maintainable URDF files
- Best practices for creating effective robot descriptions
- How to work with URDF in ROS 2 systems

## Next Steps

Continue to [Mini-Projects and Exercises](./exercises.md) to apply all the concepts learned in this module through practical projects and exercises.