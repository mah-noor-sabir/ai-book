---
sidebar_position: 4
---

# Creating Your First ROS 2 Node

## Overview

In this chapter, you'll create your first ROS 2 node using Python and the rclpy client library. This hands-on experience will solidify your understanding of ROS 2 architecture and provide a foundation for more complex robotic applications.

## Prerequisites

Before starting this chapter, ensure you have:
- Completed the [Installation and Setup](./installation.md) chapter
- Understanding of basic Python programming
- Familiarity with command-line operations
- Basic understanding of ROS 2 architecture from the previous chapter

## Setting Up Your Workspace

First, let's create a dedicated workspace for our learning projects:

```bash
# Create the workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Create a package for our first node
ros2 pkg create --build-type ament_python first_robot_package --dependencies rclpy std_msgs
```

This creates a Python package with the necessary structure and dependencies.

## Understanding the Package Structure

After creating the package, explore the generated structure:

```
first_robot_package/
├── first_robot_package/
│   ├── __init__.py
│   └── first_node.py
├── package.xml
├── setup.cfg
├── setup.py
└── test/
    ├── __init__.py
    └── test_copyright.py
    └── test_flake8.py
    └── test_pep257.py
```

## Creating a Simple Publisher Node

Let's create our first node that publishes messages to a topic. Open the file `~/ros2_ws/src/first_robot_package/first_robot_package/first_node.py` and replace its contents:

```python
#!/usr/bin/env python3
"""
Simple Publisher Node

This node publishes "Hello World" messages to a topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """A simple publisher node that sends messages to a topic."""

    def __init__(self):
        """Initialize the publisher node."""
        super().__init__('simple_publisher')

        # Create a publisher with a String message type
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Set a timer to publish messages at 2 Hz
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to keep track of messages
        self.i = 0

        self.get_logger().info('Simple Publisher Node has been started')

    def timer_callback(self):
        """Callback function that publishes messages."""
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher.publish(msg)

        # Log the message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter
        self.i += 1


def main(args=None):
    """Main function to run the node."""
    rclpy.init(args=args)

    simple_publisher = SimplePublisher()

    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        simple_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating a Simple Subscriber Node

Now let's create a subscriber node that receives messages from our publisher. Create a new file `~/ros2_ws/src/first_robot_package/first_robot_package/subscriber_node.py`:

```python
#!/usr/bin/env python3
"""
Simple Subscriber Node

This node subscribes to messages from a topic and logs them.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """A simple subscriber node that receives messages from a topic."""

    def __init__(self):
        """Initialize the subscriber node."""
        super().__init__('simple_subscriber')

        # Create a subscription to the 'chatter' topic
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)  # QoS depth

        # Make sure subscription is not destroyed
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Simple Subscriber Node has been started')

    def listener_callback(self, msg):
        """Callback function that processes incoming messages."""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Main function to run the node."""
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()

    try:
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        simple_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Building and Running Your Nodes

### Step 1: Build Your Package
```bash
cd ~/ros2_ws
colcon build --packages-select first_robot_package
```

### Step 2: Source Your Workspace
```bash
source install/setup.bash
```

### Step 3: Run the Publisher Node
Open a new terminal, source the workspace, and run:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run first_robot_package first_node
```

### Step 4: Run the Subscriber Node
Open another terminal, source the workspace, and run:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run first_robot_package subscriber_node
```

You should now see the publisher sending messages and the subscriber receiving them!

## Understanding the Code

### Node Initialization
```python
class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
```
This creates a new node with the name 'simple_publisher'.

### Creating Publishers
```python
self.publisher = self.create_publisher(String, 'chatter', 10)
```
Creates a publisher that sends String messages to the 'chatter' topic with a queue size of 10.

### Creating Timers
```python
self.timer = self.create_timer(timer_period, self.timer_callback)
```
Creates a timer that calls the callback function every 0.5 seconds.

### Creating Subscriptions
```python
self.subscription = self.create_subscription(
    String,
    'chatter',
    self.listener_callback,
    10)
```
Creates a subscription to the 'chatter' topic that calls the listener_callback when messages arrive.

## Extending Your First Node

Let's create a more advanced node that demonstrates multiple concepts. Create `~/ros2_ws/src/first_robot_package/first_robot_package/advanced_node.py`:

```python
#!/usr/bin/env python3
"""
Advanced Node Example

This node demonstrates multiple ROS 2 concepts:
- Multiple publishers and subscribers
- Parameters
- Services
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Int32
from example_interfaces.srv import AddTwoInts


class AdvancedNode(Node):
    """An advanced node demonstrating multiple ROS 2 concepts."""

    def __init__(self):
        """Initialize the advanced node."""
        super().__init__('advanced_node')

        # Declare parameters with default values
        self.declare_parameter('message_rate', 1)
        self.declare_parameter('robot_name', 'my_robot')

        # Get parameter values
        self.message_rate = self.get_parameter('message_rate').value
        self.robot_name = self.get_parameter('robot_name').value

        # Create QoS profile
        qos_profile = QoSProfile(depth=10)

        # Create publishers
        self.message_publisher = self.create_publisher(String, 'robot_message', qos_profile)
        self.counter_publisher = self.create_publisher(Int32, 'robot_counter', qos_profile)

        # Create subscriptions
        self.message_subscriber = self.create_subscription(
            String,
            'command',
            self.command_callback,
            qos_profile
        )

        # Create timer
        self.counter = 0
        self.timer = self.create_timer(1.0 / self.message_rate, self.timer_callback)

        # Create service
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

        self.get_logger().info(f'Advanced Node started for robot: {self.robot_name}')

    def timer_callback(self):
        """Publish periodic messages."""
        # Publish message
        msg = String()
        msg.data = f'Robot {self.robot_name} status: operational - {self.counter}'
        self.message_publisher.publish(msg)

        # Publish counter
        counter_msg = Int32()
        counter_msg.data = self.counter
        self.counter_publisher.publish(counter_msg)

        self.counter += 1

    def command_callback(self, msg):
        """Handle incoming commands."""
        self.get_logger().info(f'Received command: {msg.data}')

        # Process command based on content
        if 'reset' in msg.data.lower():
            self.counter = 0
            self.get_logger().info('Counter reset to 0')

    def add_two_ints_callback(self, request, response):
        """Service callback to add two integers."""
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    """Main function to run the node."""
    rclpy.init(args=args)

    advanced_node = AdvancedNode()

    try:
        rclpy.spin(advanced_node)
    except KeyboardInterrupt:
        pass
    finally:
        advanced_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Running the Advanced Node

### Step 1: Build the Updated Package
```bash
cd ~/ros2_ws
colcon build --packages-select first_robot_package
source install/setup.bash
```

### Step 2: Run the Advanced Node
```bash
ros2 run first_robot_package advanced_node
```

### Step 3: Test the Service
In another terminal:
```bash
source ~/ros2_ws/install/setup.bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

## Node Management Commands

### List Running Nodes
```bash
ros2 node list
```

### Get Information About a Node
```bash
ros2 node info /simple_publisher
```

### Echo Topics
```bash
ros2 topic echo /chatter
```

### Publish to a Topic
```bash
ros2 topic pub /command std_msgs/msg/String "data: 'reset'"
```

## Debugging Tips

### 1. Check Node Status
```bash
# List all nodes
ros2 node list

# Check specific node info
ros2 node info /your_node_name
```

### 2. Monitor Topics
```bash
# List all topics
ros2 topic list

# Check topic info
ros2 topic info /topic_name

# Echo messages
ros2 topic echo /topic_name
```

### 3. Use Logging Effectively
```python
# Different log levels
self.get_logger().debug('Debug message')
self.get_logger().info('Info message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal message')
```

## Common Pitfalls and Solutions

### 1. Node Names Conflicts
**Problem**: Multiple nodes with the same name
**Solution**: Use unique names or namespaces

### 2. Topic Names Issues
**Problem**: Publishers and subscribers not connecting
**Solution**: Verify topic names match exactly

### 3. Import Errors
**Problem**: Cannot import ROS 2 modules
**Solution**: Ensure workspace is sourced and dependencies are installed

### 4. Permission Issues
**Problem**: Cannot create nodes or topics
**Solution**: Check ROS 2 environment variables and permissions

## Mini-Exercise: Temperature Sensor Node

Create a temperature sensor node that:
1. Publishes random temperature values every 2 seconds
2. Subscribes to a "temperature_threshold" topic
3. Logs warnings when temperature exceeds the threshold

Try implementing this on your own before checking the solution below.

<details>
<summary>Click here for the solution</summary>

```python
#!/usr/bin/env python3
"""
Temperature Sensor Node

Simulates a temperature sensor that publishes readings and monitors thresholds.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class TemperatureSensor(Node):
    """Temperature sensor node with threshold monitoring."""

    def __init__(self):
        super().__init__('temperature_sensor')

        # Create publisher for temperature readings
        self.temp_publisher = self.create_publisher(Float32, 'temperature_readings', 10)

        # Create subscriber for threshold
        self.threshold_subscriber = self.create_subscription(
            Float32,
            'temperature_threshold',
            self.threshold_callback,
            10
        )

        # Initialize threshold
        self.threshold = 25.0  # Default threshold

        # Create timer for temperature readings
        self.timer = self.create_timer(2.0, self.publish_temperature)

        self.get_logger().info('Temperature Sensor Node started')

    def threshold_callback(self, msg):
        """Update temperature threshold."""
        self.threshold = msg.data
        self.get_logger().info(f'Temperature threshold updated to: {self.threshold}')

    def publish_temperature(self):
        """Publish random temperature reading."""
        # Generate random temperature around 20°C with some variation
        temperature = 20.0 + random.uniform(-5, 15)

        msg = Float32()
        msg.data = temperature
        self.temp_publisher.publish(msg)

        # Check if temperature exceeds threshold
        if temperature > self.threshold:
            self.get_logger().warn(f'TEMPERATURE ALERT: {temperature}°C exceeds threshold {self.threshold}°C')
        else:
            self.get_logger().info(f'Temperature: {temperature}°C (threshold: {self.threshold}°C)')


def main(args=None):
    rclpy.init(args=args)

    temp_sensor = TemperatureSensor()

    try:
        rclpy.spin(temp_sensor)
    except KeyboardInterrupt:
        pass
    finally:
        temp_sensor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

</details>

## Summary

In this chapter, you've learned to:
- Create ROS 2 packages and nodes
- Implement publishers and subscribers
- Use parameters, services, and timers
- Debug common issues
- Apply best practices for node development

## Next Steps

Continue to [Topics and Publishers/Subscribers](./topics.md) to dive deeper into topic-based communication and learn advanced patterns for effective robot communication.