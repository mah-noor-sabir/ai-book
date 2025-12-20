---
sidebar_position: 6
---

# Services and Clients

## Overview

While topics enable asynchronous communication in ROS 2, **services** provide synchronous request-response communication. Services are essential for operations that require immediate responses, such as configuration changes, data retrieval, or triggering specific actions. In this chapter, you'll learn to implement and use services effectively in robotic applications.

## Understanding Services

### What Are Services?

Services in ROS 2 implement a **client-server** communication pattern where:

- **Service Clients** send requests to a service
- **Service Servers** receive requests, process them, and send responses
- Communication is synchronous - the client waits for the response
- Each service has a specific request and response message type

### When to Use Services

Use services for:
- Configuration operations (setting parameters, calibrating sensors)
- Data retrieval (getting maps, current state)
- Action triggering (stopping robot, saving data)
- Operations requiring immediate confirmation

### Service vs. Topic Comparison

| Aspect | Topics | Services |
|--------|--------|----------|
| Communication | Asynchronous | Synchronous |
| Pattern | Publish-Subscribe | Request-Response |
| Timing | Decoupled | Client waits for response |
| Use Case | Continuous data flow | On-demand operations |

## Service Message Types

### Built-in Service Types

ROS 2 provides standard service types in `example_interfaces` and other packages:

- `example_interfaces/srv/AddTwoInts` - Simple addition example
- `std_srvs/srv/Empty` - Services that require no parameters
- `std_srvs/srv/SetBool` - Setting boolean parameters
- `std_srvs/srv/Trigger` - Triggering operations

### Service Definition Structure

Service files (`.srv`) have three parts separated by `---`:

```
# Request part (before ---)
string name
int32 value

---
# Response part (after ---)
bool success
string message
```

## Creating Service Servers

### Basic Service Server

```python
from example_interfaces.srv import AddTwoInts

class BasicServiceServer(Node):
    def __init__(self):
        super().__init__('basic_service_server')

        # Create service server
        self.srv = self.create_service(
            AddTwoInts,           # Service type
            'add_two_ints',       # Service name
            self.add_callback     # Callback function
        )
        self.get_logger().info('Service server created')

    def add_callback(self, request, response):
        """Process request and return response."""
        result = request.a + request.b
        response.sum = result
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

### Advanced Service Server with Error Handling

```python
from example_interfaces.srv import AddTwoInts
import math

class AdvancedServiceServer(Node):
    def __init__(self):
        super().__init__('advanced_service_server')

        self.srv = self.create_service(
            AddTwoInts, 'safe_add', self.safe_add_callback)

        self.get_logger().info('Advanced service server created')

    def safe_add_callback(self, request, response):
        """Safely add two integers with overflow protection."""
        try:
            # Check for potential overflow
            if (request.a > 0 and request.b > 0 and
                request.a > math.inf - request.b):
                response.sum = math.inf
                return response

            if (request.a < 0 and request.b < 0 and
                request.a < -math.inf - request.b):
                response.sum = -math.inf
                return response

            response.sum = request.a + request.b
            return response

        except Exception as e:
            self.get_logger().error(f'Service error: {e}')
            response.sum = 0  # Default response on error
            return response
```

## Creating Service Clients

### Basic Service Client

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class BasicServiceClient(Node):
    def __init__(self):
        super().__init__('basic_service_client')

        # Create client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Service client created')

    def send_request(self, a, b):
        """Send request and wait for response."""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call service synchronously
        future = self.cli.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
            return response.sum
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return None
```

### Asynchronous Service Client

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


class AsyncServiceClient(Node):
    def __init__(self):
        super().__init__('async_service_client')

        # Use reentrant callback group for multiple services
        self.callback_group = ReentrantCallbackGroup()

        self.cli = self.create_client(
            AddTwoInts, 'add_two_ints', callback_group=self.callback_group)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request_async(self, a, b, callback=None):
        """Send request asynchronously."""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.cli.call_async(request)
        future.add_done_callback(
            lambda future: self.response_callback(future, callback))

        return future

    def response_callback(self, future, user_callback=None):
        """Handle service response."""
        try:
            response = future.result()
            self.get_logger().info(f'Received response: {response.sum}')

            if user_callback:
                user_callback(response.sum)

        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
```

## Custom Service Types

### Creating Custom Services

1. Create a `srv` directory in your package
2. Create a `.srv` file with your service definition

**Example: `srv/RobotControl.srv`**
```
# Request - Command for the robot
string command
float64[] parameters

---
# Response - Result of the command
bool success
string message
int32 error_code
```

### Using Custom Services

After creating custom services, you need to:
1. Update `package.xml` to include `rosidl_default_generators`
2. Update `setup.py` to include the service files
3. Rebuild the package

```python
# Import your custom service
from your_package.srv import RobotControl

class RobotControlServer(Node):
    def __init__(self):
        super().__init__('robot_control_server')

        self.srv = self.create_service(
            RobotControl, 'robot_control', self.control_callback)

    def control_callback(self, request, response):
        """Handle robot control commands."""
        self.get_logger().info(f'Received command: {request.command}')

        # Process the command
        if request.command == 'move':
            # Process move command with parameters
            x, y, theta = request.parameters
            success = self.execute_move(x, y, theta)
            response.success = success
            response.message = 'Move command processed' if success else 'Move failed'
            response.error_code = 0 if success else 1
        else:
            response.success = False
            response.message = f'Unknown command: {request.command}'
            response.error_code = -1

        return response
```

## Practical Example: Robot Configuration Service

Let's create a comprehensive example that demonstrates services in a real-world scenario:

```python
#!/usr/bin/env python3
"""
Robot Configuration Service

Provides services for robot configuration and status management.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from example_interfaces.srv import SetBool, Trigger
from std_msgs.msg import String
from std_srvs.srv import SetBool as StdSetBool, Trigger as StdTrigger
import json
from datetime import datetime


class RobotConfigService(Node):
    """Service server for robot configuration and management."""

    def __init__(self):
        super().__init__('robot_config_service')

        # Service servers
        self.enable_srv = self.create_service(
            StdSetBool, 'enable_robot', self.enable_robot_callback)
        self.reset_srv = self.create_service(
            StdTrigger, 'reset_robot', self.reset_robot_callback)
        self.save_config_srv = self.create_service(
            StdTrigger, 'save_config', self.save_config_callback)
        self.load_config_srv = self.create_service(
            StdTrigger, 'load_config', self.load_config_callback)

        # Status publisher
        self.status_pub = self.create_publisher(String, 'robot_status', 10)

        # Robot state
        self.enabled = False
        self.config = {
            'last_reset': None,
            'operation_count': 0,
            'parameters': {}
        }

        self.get_logger().info('Robot Configuration Service started')

    def enable_robot_callback(self, request, response):
        """Enable or disable robot operations."""
        old_state = self.enabled
        self.enabled = request.data

        response.success = True
        response.message = f'Robot {"enabled" if self.enabled else "disabled"}'

        if old_state != self.enabled:
            self.get_logger().info(f'Robot {"enabled" if self.enabled else "disabled"}')
            self.publish_status()

        return response

    def reset_robot_callback(self, request, response):
        """Reset robot to initial state."""
        if not self.enabled:
            response.success = False
            response.message = 'Cannot reset: Robot is disabled'
            return response

        try:
            # Reset operations
            self.config['last_reset'] = datetime.now().isoformat()
            self.config['operation_count'] = 0
            # Add more reset operations as needed

            response.success = True
            response.message = 'Robot reset successfully'
            self.get_logger().info('Robot reset performed')

            self.publish_status()
            return response

        except Exception as e:
            response.success = False
            response.message = f'Reset failed: {str(e)}'
            self.get_logger().error(f'Reset error: {e}')
            return response

    def save_config_callback(self, request, response):
        """Save current configuration to file."""
        try:
            config_file = '/tmp/robot_config.json'
            with open(config_file, 'w') as f:
                json.dump(self.config, f, indent=2)

            response.success = True
            response.message = f'Configuration saved to {config_file}'
            self.get_logger().info(f'Configuration saved to {config_file}')

            return response

        except Exception as e:
            response.success = False
            response.message = f'Config save failed: {str(e)}'
            self.get_logger().error(f'Config save error: {e}')
            return response

    def load_config_callback(self, request, response):
        """Load configuration from file."""
        try:
            config_file = '/tmp/robot_config.json'
            with open(config_file, 'r') as f:
                self.config = json.load(f)

            response.success = True
            response.message = f'Configuration loaded from {config_file}'
            self.get_logger().info(f'Configuration loaded from {config_file}')

            return response

        except FileNotFoundError:
            response.success = False
            response.message = f'Config file not found: {config_file}'
            self.get_logger().warn(f'Config file not found: {config_file}')
            return response
        except Exception as e:
            response.success = False
            response.message = f'Config load failed: {str(e)}'
            self.get_logger().error(f'Config load error: {e}')
            return response

    def publish_status(self):
        """Publish current robot status."""
        status_msg = String()
        status_msg.data = json.dumps({
            'enabled': self.enabled,
            'config': self.config
        })
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotConfigService()

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

## Service Client Example

Here's a client that uses the robot configuration services:

```python
#!/usr/bin/env python3
"""
Robot Configuration Client

Client for the robot configuration services.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
import time


class RobotConfigClient(Node):
    """Client for robot configuration services."""

    def __init__(self):
        super().__init__('robot_config_client')

        # Create clients
        self.enable_client = self.create_client(SetBool, 'enable_robot')
        self.reset_client = self.create_client(Trigger, 'reset_robot')
        self.save_client = self.create_client(Trigger, 'save_config')
        self.load_client = self.create_client(Trigger, 'load_config')

        # Wait for services
        while not all([
            self.enable_client.wait_for_service(timeout_sec=1.0),
            self.reset_client.wait_for_service(timeout_sec=1.0),
            self.save_client.wait_for_service(timeout_sec=1.0),
            self.load_client.wait_for_service(timeout_sec=1.0)
        ]):
            self.get_logger().info('Waiting for services...')

        self.get_logger().info('All services available')

    def enable_robot(self, enable):
        """Enable or disable the robot."""
        request = SetBool.Request()
        request.data = enable

        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
            self.get_logger().info(f'Enable response: {response.success}, {response.message}')
            return response
        except Exception as e:
            self.get_logger().error(f'Enable service call failed: {e}')
            return None

    def reset_robot(self):
        """Reset the robot."""
        request = Trigger.Request()

        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
            self.get_logger().info(f'Reset response: {response.success}, {response.message}')
            return response
        except Exception as e:
            self.get_logger().error(f'Reset service call failed: {e}')
            return None

    def save_config(self):
        """Save robot configuration."""
        request = Trigger.Request()

        future = self.save_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
            self.get_logger().info(f'Save response: {response.success}, {response.message}')
            return response
        except Exception as e:
            self.get_logger().error(f'Save service call failed: {e}')
            return None

    def demo_sequence(self):
        """Demonstrate service usage sequence."""
        self.get_logger().info('Starting robot configuration demo...')

        # Enable robot
        self.enable_robot(True)

        # Wait a bit
        time.sleep(1)

        # Reset robot
        self.reset_robot()

        # Wait a bit
        time.sleep(1)

        # Save configuration
        self.save_config()

        self.get_logger().info('Demo completed')


def main(args=None):
    rclpy.init(args=args)
    client = RobotConfigClient()

    # Run demo sequence
    client.demo_sequence()

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Service Monitoring and Debugging

### Command Line Tools

```bash
# List all services
ros2 service list

# Get info about a specific service
ros2 service info /service_name

# Call a service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# Get service type
ros2 service type /service_name
```

### Programmatic Service Discovery

```python
def check_service_availability(self, service_name, service_type):
    """Check if a service is available."""
    client = self.create_client(service_type, service_name)

    # Wait for service with timeout
    if client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info(f'Service {service_name} is available')
        return True
    else:
        self.get_logger().warn(f'Service {service_name} is not available')
        return False
```

## Best Practices

### 1. Service Design
- Use services for operations that require immediate responses
- Keep service requests/responses small and efficient
- Design clear, specific service interfaces
- Use appropriate service types for the operation

### 2. Error Handling
```python
def robust_service_callback(self, request, response):
    """Service callback with comprehensive error handling."""
    try:
        # Validate input
        if request.a < 0 or request.b < 0:
            response.success = False
            response.message = 'Negative values not allowed'
            return response

        # Process request
        result = self.process_request(request)

        # Set response
        response.success = True
        response.result = result
        response.message = 'Success'

        return response

    except ValueError as e:
        response.success = False
        response.message = f'Invalid input: {e}'
        return response
    except Exception as e:
        self.get_logger().error(f'Service error: {e}')
        response.success = False
        response.message = f'Internal error: {e}'
        return response
```

### 3. Service Naming
- Use descriptive names: `enable_robot`, `get_map`, `save_config`
- Follow consistent naming patterns
- Include domain context when appropriate: `navigation/set_goal`

### 4. Asynchronous Service Calls
For better performance, use asynchronous service calls when possible:

```python
def async_service_call(self, request):
    """Make asynchronous service call."""
    future = self.client.call_async(request)
    future.add_done_callback(self.handle_service_response)
    return future

def handle_service_response(self, future):
    """Handle service response in callback."""
    try:
        response = future.result()
        # Process response
    except Exception as e:
        self.get_logger().error(f'Service call failed: {e}')
```

## Mini-Exercise: Map Service Node

Create a map service that:
1. Provides services to get, save, and load maps
2. Uses custom service messages for map operations
3. Includes proper error handling
4. Demonstrates both server and client implementations

<details>
<summary>Click here for the solution</summary>

**Service definition file (`srv/MapOperation.srv`):**
```
# Request
string operation  # "get", "save", "load"
string map_name
string map_data   # For save operation

---
# Response
bool success
string message
string map_data   # For get operation
int32 error_code
```

**Map Service Server:**
```python
#!/usr/bin/env python3
"""
Map Service Server

Provides map-related services for navigation.
"""

import rclpy
from rclpy.node import Node
from your_package.srv import MapOperation  # You would create this service
import os
import json


class MapServiceServer(Node):
    def __init__(self):
        super().__init__('map_service_server')

        # In a real implementation, you would define the MapOperation service
        # For this example, we'll use AddTwoInts as a placeholder
        # You would replace this with your actual service type
        from example_interfaces.srv import AddTwoInts

        # Use a simple service for demonstration
        self.map_service = self.create_service(
            AddTwoInts, 'get_map_info', self.map_info_callback)

        # Simulated map storage
        self.maps = {
            'office': {'size': [10, 10], 'obstacles': []},
            'home': {'size': [8, 8], 'obstacles': []}
        }

        self.get_logger().info('Map service server started')

    def map_info_callback(self, request, response):
        """Return map information."""
        # This is a simplified example - in practice you'd implement
        # the full MapOperation service
        response.sum = request.a + request.b
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapServiceServer()

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

For a complete implementation with custom services, you would need to define the service files and update your package configuration accordingly.

</details>

## Advanced Service Patterns

### 1. Service with Multiple Clients

```python
class MultiClientService(Node):
    def __init__(self):
        super().__init__('multi_client_service')

        self.srv = self.create_service(
            SetBool, 'process_request', self.handle_request)

        # Track active requests
        self.active_requests = 0
        self.max_concurrent_requests = 3

    def handle_request(self, request, response):
        """Handle requests with concurrency control."""
        if self.active_requests >= self.max_concurrent_requests:
            response.success = False
            response.message = 'Too many concurrent requests'
            return response

        self.active_requests += 1
        try:
            # Process request
            result = self.process_request(request)
            response.success = True
            response.message = 'Request processed'
        except Exception as e:
            response.success = False
            response.message = f'Error: {e}'
        finally:
            self.active_requests -= 1

        return response
```

### 2. Service with Timeout

```python
def call_service_with_timeout(self, request, timeout_sec=5.0):
    """Call service with timeout."""
    future = self.client.call_async(request)

    # Wait for result with timeout
    timer = self.create_timer(timeout_sec, lambda: None)
    try:
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.done():
            return future.result()
        else:
            raise TimeoutError("Service call timed out")
    finally:
        timer.destroy()
```

## Summary

In this chapter, you've learned:
- How services enable synchronous request-response communication
- How to create service servers and clients
- When to use services vs. topics
- Best practices for service design and implementation
- Advanced patterns for complex service scenarios

## Next Steps

Continue to [Introduction to URDF](./urdf-intro.md) to learn about robot description and modeling, which is crucial for robot simulation and visualization.