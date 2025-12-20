---
sidebar_position: 5
---

# Topics and Publishers/Subscribers

## Overview

Topics form the backbone of asynchronous communication in ROS 2. Understanding how to effectively use topics, publishers, and subscribers is crucial for building responsive and scalable robotic systems. In this chapter, we'll explore advanced concepts and patterns for topic-based communication.

## Understanding Topics

### What Are Topics?

Topics are named buses over which nodes exchange messages. They implement a **publish-subscribe** communication pattern where:

- **Publishers** send data to a topic
- **Subscribers** receive data from a topic
- Multiple publishers and subscribers can use the same topic
- Communication is asynchronous and decoupled

### Key Characteristics

- **Asynchronous**: Publishers and subscribers don't need to run simultaneously
- **Anonymous**: Publishers don't know who subscribes, subscribers don't know who publishes
- **Typed**: Each topic has a specific message type
- **Named**: Topics have unique names within the ROS graph

## Message Types

### Built-in Message Types

ROS 2 provides many standard message types in the `std_msgs` package:

- `std_msgs/msg/String` - Text messages
- `std_msgs/msg/Int32`, `std_msgs/msg/Float64` - Numeric values
- `std_msgs/msg/Bool` - Boolean values
- `sensor_msgs/msg/LaserScan` - LIDAR data
- `sensor_msgs/msg/Image` - Camera images
- `geometry_msgs/msg/Twist` - Velocity commands
- `nav_msgs/msg/Odometry` - Robot position and velocity

### Creating Custom Messages

To create a custom message, create a `.msg` file in your package's `msg` directory:

**Example: `msg/RobotStatus.msg`**
```
string robot_name
int32 battery_level
bool is_charging
float64[] joint_positions
```

## Publishers

### Basic Publisher Creation

```python
from std_msgs.msg import String

# Create publisher
publisher = self.create_publisher(String, 'topic_name', qos_profile=10)

# Publish a message
msg = String()
msg.data = 'Hello, ROS 2!'
self.publisher.publish(msg)
```

### Advanced Publisher Patterns

#### 1. Publisher with Custom QoS

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Create a custom QoS profile for high-reliability communication
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)

publisher = self.create_publisher(String, 'critical_topic', qos_profile)
```

#### 2. Conditional Publishing

```python
def publish_if_changed(self, new_data):
    """Only publish if data has changed significantly."""
    if abs(new_data - self.last_data) > self.threshold:
        msg = Float64()
        msg.data = new_data
        self.publisher.publish(msg)
        self.last_data = new_data
```

## Subscribers

### Basic Subscriber Creation

```python
from std_msgs.msg import String

# Create subscription
subscription = self.create_subscription(
    String,           # Message type
    'topic_name',     # Topic name
    self.callback,    # Callback function
    qos_profile=10    # QoS profile
)
```

### Advanced Subscriber Patterns

#### 1. Multiple Subscribers in One Node

```python
class MultiSubscriber(Node):
    def __init__(self):
        super().__init__('multi_subscriber')

        # Subscribe to multiple topics
        self.sub1 = self.create_subscription(
            String, 'topic1', self.callback1, 10)
        self.sub2 = self.create_subscription(
            Int32, 'topic2', self.callback2, 10)
        self.sub3 = self.create_subscription(
            Float64, 'topic3', self.callback3, 10)

    def callback1(self, msg):
        self.get_logger().info(f'Received from topic1: {msg.data}')

    def callback2(self, msg):
        self.get_logger().info(f'Received from topic2: {msg.data}')

    def callback3(self, msg):
        self.get_logger().info(f'Received from topic3: {msg.data}')
```

#### 2. Subscription with Custom QoS

```python
from rclpy.qos import QoSProfile, DurabilityPolicy

# Create subscription for latched topic (transient local)
qos_profile = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

subscription = self.create_subscription(
    String, 'latched_topic', self.callback, qos_profile)
```

## Quality of Service (QoS) Settings

### Reliability Policy
- `RELIABLE`: All messages are delivered (like TCP)
- `BEST_EFFORT`: Messages may be lost (like UDP)

### Durability Policy
- `VOLATILE`: Only new messages after subscription
- `TRANSIENT_LOCAL`: All messages including past ones (latching)

### History Policy
- `KEEP_LAST`: Store N most recent messages
- `KEEP_ALL`: Store all messages (limited by memory)

### Example QoS Configurations

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# High-frequency sensor data (best effort, keep last 5)
sensor_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST
)

# Critical commands (reliable, keep last 1)
command_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)

# Static map (reliable, keep all, transient local)
map_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_ALL,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)
```

## Topic Design Patterns

### 1. Sensor Data Broadcasting

```python
class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher = self.create_publisher(
            sensor_msgs.msg.LaserScan, 'laser_scan', 10)

    def sensor_callback(self, sensor_data):
        """Process sensor data and publish to topic."""
        msg = sensor_msgs.msg.LaserScan()
        # Fill message with sensor data
        self.publisher.publish(msg)
```

### 2. State Publishing

```python
class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        self.odom_publisher = self.create_publisher(
            nav_msgs.msg.Odometry, 'odom', 10)
        self.joint_publisher = self.create_publisher(
            sensor_msgs.msg.JointState, 'joint_states', 10)
```

### 3. Command Distribution

```python
class CommandRouter(Node):
    def __init__(self):
        super().__init__('command_router')
        self.cmd_vel_sub = self.create_subscription(
            geometry_msgs.msg.Twist, 'cmd_vel_input', self.route_command, 10)

        # Multiple publishers for different systems
        self.base_controller_pub = self.create_publisher(
            geometry_msgs.msg.Twist, 'base_controller/cmd_vel', 10)
        self.simulator_pub = self.create_publisher(
            geometry_msgs.msg.Twist, 'gazebo/cmd_vel', 10)

    def route_command(self, msg):
        """Route commands to multiple destinations."""
        self.base_controller_pub.publish(msg)
        self.simulator_pub.publish(msg)
```

## Practical Example: Sensor Fusion Node

Let's create a comprehensive example that demonstrates multiple topics and advanced patterns:

```python
#!/usr/bin/env python3
"""
Sensor Fusion Node

This node subscribes to multiple sensor topics and publishes fused data.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, MagneticField
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np


class SensorFusionNode(Node):
    """Fuses data from multiple sensors."""

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # QoS profiles for different sensor types
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        cmd_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Subscriptions to various sensors
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, sensor_qos)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, sensor_qos)
        self.mag_sub = self.create_subscription(
            MagneticField, 'imu/mag', self.mag_callback, sensor_qos)

        # Publisher for fused data
        self.fused_publisher = self.create_publisher(
            Float32MultiArray, 'fused_sensor_data', cmd_qos)

        # Store sensor data
        self.laser_data = None
        self.imu_data = None
        self.mag_data = None

        # Timer for fusion processing
        self.fusion_timer = self.create_timer(0.1, self.fuse_sensors)

        self.get_logger().info('Sensor Fusion Node initialized')

    def laser_callback(self, msg):
        """Handle laser scan data."""
        self.laser_data = msg
        self.get_logger().debug('Received laser data')

    def imu_callback(self, msg):
        """Handle IMU data."""
        self.imu_data = msg
        self.get_logger().debug('Received IMU data')

    def mag_callback(self, msg):
        """Handle magnetometer data."""
        self.mag_data = msg
        self.get_logger().debug('Received magnetometer data')

    def fuse_sensors(self):
        """Fuse sensor data and publish results."""
        if not all([self.laser_data, self.imu_data, self.mag_data]):
            return  # Wait for all sensors

        # Perform sensor fusion (simplified example)
        fused_msg = Float32MultiArray()

        # Extract relevant data
        obstacle_distance = min(self.laser_data.ranges) if self.laser_data.ranges else float('inf')
        linear_accel = [self.imu_data.linear_acceleration.x,
                        self.imu_data.linear_acceleration.y,
                        self.imu_data.linear_acceleration.z]
        magnetic_field = [self.mag_data.magnetic_field.x,
                         self.mag_data.magnetic_field.y,
                         self.mag_data.magnetic_field.z]

        # Create fused data array
        fused_msg.data = [
            obstacle_distance,  # Index 0: Obstacle distance
            linear_accel[0],    # Index 1: X acceleration
            linear_accel[1],    # Index 2: Y acceleration
            linear_accel[2],    # Index 3: Z acceleration
            magnetic_field[0],  # Index 4: X magnetic field
            magnetic_field[1],  # Index 5: Y magnetic field
            magnetic_field[2]   # Index 6: Z magnetic field
        ]

        self.fused_publisher.publish(fused_msg)
        self.get_logger().info(f'Fused data published: obstacle={obstacle_distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()

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

## Topic Monitoring and Debugging

### Command Line Tools

```bash
# List all topics
ros2 topic list

# Get info about a specific topic
ros2 topic info /topic_name

# Echo messages from a topic
ros2 topic echo /topic_name

# Echo with specific number of messages
ros2 topic echo /topic_name --field data -n 5

# Find publishers/subscribers
ros2 topic list -t  # Show types
ros2 topic info /chatter  # Show connections
```

### Programmatic Monitoring

```python
def check_topic_status(self):
    """Check if publishers are active for a topic."""
    if self.publisher.get_subscription_count() > 0:
        self.get_logger().info('Subscribers are active')
    else:
        self.get_logger().warn('No subscribers connected')
```

## Best Practices

### 1. Topic Naming Conventions
- Use descriptive, lowercase names with underscores
- Follow hierarchical structure: `/robot_name/sensor_type/data_type`
- Example: `/turtlebot3/lidar/scan`, `/ur5/joint_states`, `/camera/rgb/image_raw`

### 2. Message Design
- Keep messages small for high-frequency topics
- Use appropriate data types (avoid strings for numeric data)
- Consider bandwidth and processing requirements

### 3. QoS Selection
- Use `BEST_EFFORT` for sensor data where some loss is acceptable
- Use `RELIABLE` for commands and critical data
- Use `TRANSIENT_LOCAL` for static data that new subscribers should receive

### 4. Error Handling
```python
def safe_publish(self, msg):
    """Safely publish a message with error handling."""
    try:
        self.publisher.publish(msg)
    except Exception as e:
        self.get_logger().error(f'Failed to publish message: {e}')
```

## Mini-Exercise: Navigation Safety Node

Create a safety node that:
1. Subscribes to LIDAR data and velocity commands
2. Monitors for obstacles in the robot's path
3. Publishes emergency stop commands when obstacles are detected
4. Uses appropriate QoS settings for safety-critical communication

<details>
<summary>Click here for the solution</summary>

```python
#!/usr/bin/env python3
"""
Navigation Safety Node

Monitors LIDAR data and stops robot if obstacles are detected in path.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


class SafetyNode(Node):
    """Safety node that monitors for obstacles and stops robot if needed."""

    def __init__(self):
        super().__init__('safety_node')

        # QoS for safety-critical messages
        safety_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, safety_qos)
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, safety_qos)

        # Publisher for safety commands
        self.safety_cmd_pub = self.create_publisher(
            Twist, 'safety_cmd_vel', safety_qos)

        # Parameters
        self.declare_parameter('safety_distance', 0.5)  # meters
        self.declare_parameter('forward_angle_range', 30)  # degrees

        self.safety_distance = self.get_parameter('safety_distance').value
        self.forward_angle_range = self.get_parameter('forward_angle_range').value

        # State
        self.last_cmd = Twist()
        self.obstacle_detected = False

        self.get_logger().info('Safety Node initialized')

    def cmd_callback(self, msg):
        """Store the latest command for potential override."""
        self.last_cmd = msg

    def scan_callback(self, msg):
        """Process LIDAR data to detect obstacles."""
        # Calculate angle range for forward detection
        min_angle = -self.forward_angle_range / 2
        max_angle = self.forward_angle_range / 2

        # Convert to appropriate indices
        angle_increment = msg.angle_increment
        min_idx = int((min_angle - msg.angle_min) / angle_increment)
        max_idx = int((max_angle - msg.angle_min) / angle_increment)

        # Ensure indices are within bounds
        min_idx = max(0, min_idx)
        max_idx = min(len(msg.ranges), max_idx)

        # Check for obstacles in forward direction
        forward_ranges = msg.ranges[min_idx:max_idx]
        min_distance = min([r for r in forward_ranges if 0 < r < float('inf')], default=float('inf'))

        # Check if obstacle is too close
        if min_distance < self.safety_distance:
            if not self.obstacle_detected:
                self.get_logger().warn(f'OBSTACLE DETECTED: {min_distance:.2f}m away, STOPPING!')
            self.obstacle_detected = True
            self.publish_emergency_stop()
        else:
            self.obstacle_detected = False
            # Forward the original command when safe
            self.safety_cmd_pub.publish(self.last_cmd)

    def publish_emergency_stop(self):
        """Publish zero velocity to stop the robot."""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.safety_cmd_pub.publish(stop_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()

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

</details>

## Summary

In this chapter, you've learned:
- Advanced topic concepts and message types
- How to create publishers and subscribers with custom QoS
- Topic design patterns for different use cases
- Sensor fusion techniques using multiple topics
- Best practices for topic-based communication

## Next Steps

Continue to [Services and Clients](./services.md) to learn about synchronous communication patterns that complement the asynchronous topic-based communication you've mastered.