---
sidebar_position: 8
---

# Mini-Projects and Exercises

## Overview

This chapter provides hands-on exercises and mini-projects that integrate all the concepts learned in the Robotic Nervous System module. These projects will help you apply your knowledge of ROS 2 nodes, topics, services, and URDF in practical scenarios.

## Exercise 1: Smart Home Robot

### Objective
Create a robot that monitors a simulated home environment and responds to various conditions.

### Requirements
- Use topics for sensor data and commands
- Implement services for configuration
- Create a URDF model for the robot
- Handle multiple sensors and actuators

### Implementation Steps

1. **Create the package:**
```bash
cd ~/ros2_ws
ros2 pkg create --build-type ament_python smart_home_robot --dependencies rclpy std_msgs sensor_msgs geometry_msgs
```

2. **Create the robot URDF** (`smart_home_robot/urdf/home_robot.urdf`):
```xml
<?xml version="1.0"?>
<robot name="home_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.8" iyz="0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Camera -->
  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
```

3. **Create the main robot node** (`smart_home_robot/smart_home_robot/main_robot.py`):

```python
#!/usr/bin/env python3
"""
Smart Home Robot

A robot that monitors home conditions and responds to events.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import SetBool, Trigger
import random
import time


class SmartHomeRobot(Node):
    """Smart home robot that monitors and responds to home conditions."""

    def __init__(self):
        super().__init__('smart_home_robot')

        # QoS profile
        qos_profile = QoSProfile(depth=10)

        # Publishers
        self.alert_publisher = self.create_publisher(String, 'home_alerts', qos_profile)
        self.movement_publisher = self.create_publisher(Float32, 'robot_speed', qos_profile)
        self.status_publisher = self.create_publisher(String, 'robot_status', qos_profile)

        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, qos_profile)
        self.camera_sub = self.create_subscription(
            Image, 'camera/image_raw', self.camera_callback, qos_profile)

        # Services
        self.emergency_stop_srv = self.create_service(
            SetBool, 'emergency_stop', self.emergency_stop_callback)
        self.get_status_srv = self.create_service(
            Trigger, 'get_status', self.get_status_callback)

        # Robot state
        self.is_active = True
        self.obstacle_detected = False
        self.camera_alert = False
        self.last_alert_time = 0

        # Timer for periodic monitoring
        self.monitor_timer = self.create_timer(1.0, self.monitor_home)

        self.get_logger().info('Smart Home Robot initialized')

    def laser_callback(self, msg):
        """Process laser scan data."""
        if msg.ranges:
            min_distance = min([r for r in msg.ranges if 0 < r < float('inf')], default=float('inf'))
            self.obstacle_detected = min_distance < 0.5  # Alert if obstacle within 0.5m

    def camera_callback(self, msg):
        """Process camera data (simulated)."""
        # Simulate object detection
        if random.random() < 0.05:  # 5% chance of detecting something
            self.camera_alert = True
            self.trigger_alert('Movement detected by camera')

    def monitor_home(self):
        """Monitor home conditions and respond appropriately."""
        if not self.is_active:
            return

        # Check for various conditions
        if self.obstacle_detected:
            self.trigger_alert('Obstacle detected in path')
            self.movement_publisher.publish(Float32(data=0.0))  # Stop movement

        # Publish status
        status_msg = String()
        status_msg.data = f'Active: {self.is_active}, Obstacles: {self.obstacle_detected}'
        self.status_publisher.publish(status_msg)

        # Reset camera alert
        self.camera_alert = False

    def trigger_alert(self, message):
        """Trigger a home alert."""
        current_time = time.time()
        if current_time - self.last_alert_time > 2:  # Throttle alerts
            alert_msg = String()
            alert_msg.data = f'ALERT: {message}'
            self.alert_publisher.publish(alert_msg)
            self.get_logger().warn(alert_msg.data)
            self.last_alert_time = current_time

    def emergency_stop_callback(self, request, response):
        """Handle emergency stop service."""
        old_state = self.is_active
        self.is_active = not request.data  # Stop if request.data is True
        response.success = True
        response.message = f'Emergency stop {"activated" if not self.is_active else "deactivated"}'
        self.get_logger().info(response.message)
        return response

    def get_status_callback(self, request, response):
        """Return current robot status."""
        response.success = True
        response.message = f'Active: {self.is_active}, Obstacles: {self.obstacle_detected}'
        return response


def main(args=None):
    rclpy.init(args=args)
    robot = SmartHomeRobot()

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

4. **Create a sensor simulator** (`smart_home_robot/smart_home_robot/sensor_simulator.py`):

```python
#!/usr/bin/env python3
"""
Sensor Simulator for Smart Home Robot

Simulates various sensors for the home robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import random


class SensorSimulator(Node):
    """Simulates sensors for the smart home robot."""

    def __init__(self):
        super().__init__('sensor_simulator')

        # Publishers
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)

        # Timer for sensor data
        self.scan_timer = self.create_timer(0.1, self.publish_scan)

        self.get_logger().info('Sensor Simulator started')

    def publish_scan(self):
        """Publish simulated laser scan data."""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        # Set scan parameters
        msg.angle_min = -1.57  # -90 degrees
        msg.angle_max = 1.57   # 90 degrees
        msg.angle_increment = 0.1
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0

        # Generate ranges with occasional obstacles
        ranges = []
        for i in range(32):  # 32 points
            distance = 2.0 + random.uniform(-0.5, 0.5)  # Base distance with noise
            if random.random() < 0.1:  # 10% chance of obstacle
                distance = random.uniform(0.2, 0.8)  # Close obstacle
            ranges.append(distance)

        msg.ranges = ranges
        msg.intensities = [100.0] * len(ranges)  # Dummy intensities

        self.scan_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    simulator = SensorSimulator()

    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

5. **Build and test:**
```bash
cd ~/ros2_ws
colcon build --packages-select smart_home_robot
source install/setup.bash

# Run the robot
ros2 run smart_home_robot main_robot

# In another terminal, run the simulator
ros2 run smart_home_robot sensor_simulator
```

## Exercise 2: Multi-Robot Coordination System

### Objective
Create a system where multiple robots coordinate to perform tasks.

### Implementation

Create `multi_robot_system/robot_coordinator.py`:

```python
#!/usr/bin/env python3
"""
Multi-Robot Coordinator

Coordinates multiple robots for task completion.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile


class RobotCoordinator(Node):
    """Coordinates multiple robots for task completion."""

    def __init__(self):
        super().__init__('robot_coordinator')

        qos_profile = QoSProfile(depth=10)

        # Publishers for each robot
        self.robot1_cmd_pub = self.create_publisher(String, '/robot1/command', qos_profile)
        self.robot2_cmd_pub = self.create_publisher(String, '/robot2/command', qos_profile)
        self.robot3_cmd_pub = self.create_publisher(String, '/robot3/command', qos_profile)

        # Subscribers for robot status
        self.robot1_status_sub = self.create_subscription(
            String, '/robot1/status', self.robot1_status_callback, qos_profile)
        self.robot2_status_sub = self.create_subscription(
            String, '/robot2/status', self.robot2_status_callback, qos_profile)
        self.robot3_status_sub = self.create_subscription(
            String, '/robot3/status', self.robot3_status_callback, qos_profile)

        # Service to start coordination
        self.start_coordination_srv = self.create_service(
            Trigger, 'start_coordination', self.start_coordination_callback)

        # Robot states
        self.robot_states = {
            'robot1': {'status': 'idle', 'task': None},
            'robot2': {'status': 'idle', 'task': None},
            'robot3': {'status': 'idle', 'task': None}
        }

        # Task queue
        self.task_queue = [
            'patrol_area_a',
            'patrol_area_b',
            'patrol_area_c',
            'return_to_base'
        ]
        self.current_task_index = 0

        self.get_logger().info('Robot Coordinator initialized')

    def robot1_status_callback(self, msg):
        self.robot_states['robot1']['status'] = msg.data
        self.get_logger().info(f'Robot 1 status: {msg.data}')

    def robot2_status_callback(self, msg):
        self.robot_states['robot2']['status'] = msg.data
        self.get_logger().info(f'Robot 2 status: {msg.data}')

    def robot3_status_callback(self, msg):
        self.robot_states['robot3']['status'] = msg.data
        self.get_logger().info(f'Robot 3 status: {msg.data}')

    def start_coordination_callback(self, request, response):
        """Start the coordination task."""
        response.success = True
        response.message = 'Coordination started'

        if self.current_task_index < len(self.task_queue):
            self.assign_task()

        return response

    def assign_task(self):
        """Assign the next task to an available robot."""
        if self.current_task_index >= len(self.task_queue):
            self.get_logger().info('All tasks completed')
            return

        current_task = self.task_queue[self.current_task_index]

        # Find available robot
        available_robot = None
        for robot_name, state in self.robot_states.items():
            if state['status'] == 'idle' and state['task'] is None:
                available_robot = robot_name
                break

        if available_robot:
            # Assign task to robot
            self.robot_states[available_robot]['task'] = current_task

            # Send command to robot
            cmd_msg = String()
            cmd_msg.data = current_task

            if available_robot == 'robot1':
                self.robot1_cmd_pub.publish(cmd_msg)
            elif available_robot == 'robot2':
                self.robot2_cmd_pub.publish(cmd_msg)
            elif available_robot == 'robot3':
                self.robot3_cmd_pub.publish(cmd_msg)

            self.get_logger().info(f'Assigned task "{current_task}" to {available_robot}')
            self.current_task_index += 1
        else:
            self.get_logger().info('No available robots, waiting...')


def main(args=None):
    rclpy.init(args=args)
    coordinator = RobotCoordinator()

    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercise 3: Robot Simulation with Gazebo

### Objective
Integrate your robot with Gazebo simulation environment.

### Setup Steps

1. **Create a Gazebo launch file** (`smart_home_robot/launch/robot_simulation.launch.py`):

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open([
                get_package_share_directory('smart_home_robot'),
                'urdf',
                'home_robot.urdf'
            ]).read()
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'home_robot'
        ],
        output='screen'
    )

    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    return ld
```

2. **Create a simple robot controller** (`smart_home_robot/smart_home_robot/robot_controller.py`):

```python
#!/usr/bin/env python3
"""
Simple Robot Controller

Basic controller for simulated robot.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class RobotController(Node):
    """Simple controller for robot navigation."""

    def __init__(self):
        super().__init__('robot_controller')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.obstacle_distance = float('inf')
        self.obstacle_angle = 0

        self.get_logger().info('Robot Controller initialized')

    def scan_callback(self, msg):
        """Process laser scan data."""
        if msg.ranges:
            # Find minimum distance
            min_distance = min([r for r in msg.ranges if 0 < r < float('inf')], default=float('inf'))
            min_index = msg.ranges.index(min_distance) if min_distance != float('inf') else 0

            self.obstacle_distance = min_distance
            self.obstacle_angle = msg.angle_min + min_index * msg.angle_increment

    def control_loop(self):
        """Main control loop."""
        cmd = Twist()

        if self.obstacle_distance < 0.5:  # Obstacle too close
            # Stop and turn away
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 if self.obstacle_angle > 0 else -0.5
        else:
            # Move forward
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercise 4: Robot Diagnostics System

### Objective
Create a diagnostic system that monitors robot health and performance.

### Implementation

Create `robot_diagnostics/diagnostic_monitor.py`:

```python
#!/usr/bin/env python3
"""
Robot Diagnostic Monitor

Monitors robot health and performance metrics.
"""

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile
import time


class DiagnosticMonitor(Node):
    """Monitors robot diagnostic information."""

    def __init__(self):
        super().__init__('diagnostic_monitor')

        # Publishers
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Subscribers (simulated data)
        self.battery_sub = self.create_subscription(
            Float32, 'battery_level', self.battery_callback, 10)
        self.temperature_sub = self.create_subscription(
            Float32, 'cpu_temperature', self.temperature_callback, 10)

        # Timer for diagnostic publishing
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

        # Robot state
        self.battery_level = 100.0
        self.cpu_temperature = 35.0
        self.last_update = time.time()

        self.get_logger().info('Diagnostic Monitor initialized')

    def battery_callback(self, msg):
        self.battery_level = msg.data

    def temperature_callback(self, msg):
        self.cpu_temperature = msg.data

    def publish_diagnostics(self):
        """Publish diagnostic information."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        diag_array.header.frame_id = 'diagnostic_monitor'

        # Battery diagnostic
        battery_diag = DiagnosticStatus()
        battery_diag.name = 'Battery System'
        battery_diag.hardware_id = 'battery_01'

        if self.battery_level > 20:
            battery_diag.level = DiagnosticStatus.OK
            battery_diag.message = f'Battery OK: {self.battery_level:.1f}%'
        elif self.battery_level > 10:
            battery_diag.level = DiagnosticStatus.WARN
            battery_diag.message = f'Battery LOW: {self.battery_level:.1f}%'
        else:
            battery_diag.level = DiagnosticStatus.ERROR
            battery_diag.message = f'Battery CRITICAL: {self.battery_level:.1f}%'

        battery_diag.values = [
            KeyValue(key='Level', value=f'{self.battery_level:.1f}%'),
            KeyValue(key='Status', value='OK' if self.battery_level > 20 else 'LOW')
        ]

        # Temperature diagnostic
        temp_diag = DiagnosticStatus()
        temp_diag.name = 'CPU Temperature'
        temp_diag.hardware_id = 'cpu_01'

        if self.cpu_temperature < 60:
            temp_diag.level = DiagnosticStatus.OK
            temp_diag.message = f'Temperature OK: {self.cpu_temperature:.1f}°C'
        elif self.cpu_temperature < 80:
            temp_diag.level = DiagnosticStatus.WARN
            temp_diag.message = f'Temperature HIGH: {self.cpu_temperature:.1f}°C'
        else:
            temp_diag.level = DiagnosticStatus.ERROR
            temp_diag.message = f'Temperature CRITICAL: {self.cpu_temperature:.1f}°C'

        temp_diag.values = [
            KeyValue(key='Temperature', value=f'{self.cpu_temperature:.1f}°C'),
            KeyValue(key='Status', value='OK' if self.cpu_temperature < 60 else 'HIGH')
        ]

        diag_array.status = [battery_diag, temp_diag]
        self.diag_pub.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    monitor = DiagnosticMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercise 5: Complete Robot Application

### Objective
Combine all concepts into a complete robot application.

### Final Project: Autonomous Security Robot

Create a complete application that includes:
1. Robot URDF model
2. Multiple nodes for different functions
3. Topic and service communication
4. Simulation integration
5. Diagnostic monitoring

### Implementation Steps

1. **Create the main application node** (`security_robot/security_robot/main_app.py`):

```python
#!/usr/bin/env python3
"""
Autonomous Security Robot

Main application node that integrates all security robot functions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, SetBool
from rclpy.qos import QoSProfile
import math
import time


class SecurityRobot(Node):
    """Main security robot application."""

    def __init__(self):
        super().__init__('security_robot')

        qos_profile = QoSProfile(depth=10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        self.alert_pub = self.create_publisher(String, 'security_alerts', qos_profile)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile)

        # Services
        self.patrol_srv = self.create_service(
            Trigger, 'start_patrol', self.start_patrol_callback)
        self.emergency_srv = self.create_service(
            SetBool, 'emergency_mode', self.emergency_mode_callback)

        # Robot state
        self.is_patrolling = False
        self.is_emergency = False
        self.obstacle_distance = float('inf')
        self.last_alert_time = 0

        # Patrol parameters
        self.patrol_positions = [
            (1.0, 1.0, 0.0),
            (2.0, 1.0, 1.57),
            (2.0, 2.0, 3.14),
            (1.0, 2.0, -1.57)
        ]
        self.current_patrol_index = 0

        # Timer for patrol control
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Security Robot initialized')

    def scan_callback(self, msg):
        """Process laser scan data."""
        if msg.ranges:
            self.obstacle_distance = min([r for r in msg.ranges if 0 < r < float('inf')], default=float('inf'))

    def start_patrol_callback(self, request, response):
        """Start patrol mode."""
        self.is_patrolling = True
        response.success = True
        response.message = 'Patrol started'
        self.get_logger().info('Patrol mode activated')
        return response

    def emergency_mode_callback(self, request, response):
        """Handle emergency mode."""
        self.is_emergency = request.data
        response.success = True
        response.message = f'Emergency mode {"activated" if self.is_emergency else "deactivated"}'

        if self.is_emergency:
            # Stop robot in emergency
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')

        return response

    def control_loop(self):
        """Main control loop."""
        cmd = Twist()

        if self.is_emergency:
            # Stay stopped in emergency mode
            pass
        elif self.is_patrolling and not self.is_emergency:
            # Simple patrol behavior
            if self.obstacle_distance > 0.5:
                cmd.linear.x = 0.3  # Move forward
                cmd.angular.z = 0.0
            else:
                # Avoid obstacle
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5  # Turn right

            # Check for potential security alerts
            if self.obstacle_distance < 1.0 and self.obstacle_distance > 0.5:
                self.check_security_alert()
        else:
            # Stop when not patrolling
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

    def check_security_alert(self):
        """Check for security alerts."""
        current_time = time.time()
        if current_time - self.last_alert_time > 5:  # Throttle alerts
            alert_msg = String()
            alert_msg.data = f'SECURITY ALERT: Obstacle detected at {self.obstacle_distance:.2f}m'
            self.alert_pub.publish(alert_msg)
            self.get_logger().warn(alert_msg.data)
            self.last_alert_time = current_time


def main(args=None):
    rclpy.init(args=args)
    robot = SecurityRobot()

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        # Stop robot on shutdown
        stop_cmd = Twist()
        robot.cmd_vel_pub.publish(stop_cmd)
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

2. **Create a launch file** (`security_robot/launch/security_robot.launch.py`):

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Security robot main node
    security_robot = Node(
        package='security_robot',
        executable='main_app',
        name='security_robot',
        output='screen',
        parameters=[
            {'use_sim_time': False}
        ]
    )

    # Diagnostic monitor
    diagnostic_monitor = Node(
        package='security_robot',
        executable='diagnostic_monitor',
        name='diagnostic_monitor',
        output='screen'
    )

    # Sensor simulator (for testing without real hardware)
    sensor_simulator = Node(
        package='security_robot',
        executable='sensor_simulator',
        name='sensor_simulator',
        output='screen'
    )

    ld.add_action(security_robot)
    ld.add_action(diagnostic_monitor)
    ld.add_action(sensor_simulator)

    return ld
```

## Assessment Rubric

### Exercise Completion Checklist

For each exercise, ensure you can:
- [ ] Create ROS 2 packages with proper structure
- [ ] Implement nodes with publishers and subscribers
- [ ] Use services for synchronous communication
- [ ] Create valid URDF models
- [ ] Integrate multiple components
- [ ] Test functionality using ROS 2 tools
- [ ] Handle errors gracefully
- [ ] Follow ROS 2 best practices

### Advanced Challenges

1. **Real-time Performance**: Optimize your nodes for real-time performance
2. **Multi-robot Communication**: Implement communication between multiple robots
3. **Simulation Integration**: Integrate with Gazebo or other simulators
4. **Hardware Integration**: Connect to real sensors and actuators
5. **Advanced Navigation**: Implement path planning and navigation

## Summary

In this module, you've completed comprehensive exercises that demonstrate:
- ROS 2 architecture and communication patterns
- Node development and integration
- URDF modeling for robot description
- Service and topic communication
- System integration and testing

These exercises provide a solid foundation for developing complex robotic applications using ROS 2.

## Next Steps

With the Robotic Nervous System module complete, you're ready to advance to more complex topics such as:
- Advanced navigation and path planning
- Computer vision and perception
- Machine learning integration
- Multi-robot systems
- Real-world robot deployment

Continue building on these fundamentals to create sophisticated robotic applications!