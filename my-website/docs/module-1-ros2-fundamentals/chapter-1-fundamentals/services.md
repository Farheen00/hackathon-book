---
sidebar_position: 4
---

# Services and Request-Response

## Understanding Services

A **Service** in ROS 2 provides a synchronous request-response communication pattern. Unlike topics which enable asynchronous communication, services allow a client node to send a request and wait for a response from a service server. This is useful when you need a specific result from an operation.

### Key Characteristics of Services:

- **Synchronous**: The client waits for a response before continuing
- **Request-Response**: One request generates one response
- **Direct Connection**: Client directly communicates with the service server
- **Typed**: Both request and response have specific message types

## Service Message Types

Services use `.srv` files that define both request and response message types:

```
# Example service file (AddTwoInts.srv)
int64 a
int64 b
---
int64 sum
```

The part before `---` is the request, and the part after is the response.

## Creating a Service Server

Here's a complete example of a service server that adds two integers:

```python
# service_server.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server_node')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request received: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    service_server_node = ServiceServerNode()

    try:
        rclpy.spin(service_server_node)
    except KeyboardInterrupt:
        pass
    finally:
        service_server_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Service Client

Here's a service client that calls the service server:

```python
# service_client.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client_node')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        return future

def main(args=None):
    rclpy.init(args=args)
    service_client_node = ServiceClientNode()

    # Send a request
    future = service_client_node.send_request(2, 3)

    try:
        rclpy.spin_until_future_complete(service_client_node, future)
        response = future.result()
        service_client_node.get_logger().info(f'Result: {response.sum}')
    except KeyboardInterrupt:
        pass
    finally:
        service_client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running Service Server and Client

To run the service:

1. Terminal 1 (Service Server):
   ```bash
   python3 service_server.py
   ```

2. Terminal 2 (Service Client):
   ```bash
   python3 service_client.py
   ```

## Using Built-in Service Types

ROS 2 provides many built-in service types in the `example_interfaces` package:

- `AddTwoInts`: Add two integers
- `SetBool`: Set a boolean value
- `Trigger`: Trigger an action without parameters
- `GetParameters`: Get parameter values
- `SetParameters`: Set parameter values

## Service Commands

Useful command-line tools for working with services:

- `ros2 service list`: List all active services
- `ros2 service info <service_name>`: Show information about a service
- `ros2 service call <service_name> <service_type> <args>`: Call a service

Example:
```bash
# Call the add_two_ints service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 7}"
```

## Advanced Service Concepts

### Asynchronous Service Clients

For non-blocking service calls, you can use callbacks:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AsyncServiceClientNode(Node):
    def __init__(self):
        super().__init__('async_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available...')

    def send_async_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.cli.call_async(request)
        future.add_done_callback(self.service_response_callback)
        return future

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Async result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    client = AsyncServiceClientNode()

    # Send request and continue without blocking
    client.send_async_request(10, 20)

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## When to Use Services vs Topics

### Use Services When:
- You need a specific response to a request
- The operation is relatively fast
- You need guaranteed delivery
- You want synchronous behavior

### Use Topics When:
- You need to broadcast information continuously
- You don't need a response
- You want decoupled, asynchronous communication
- You're dealing with streaming data

## Best Practices for Services

1. **Keep Services Fast**: Services should respond quickly; don't perform long operations
2. **Error Handling**: Handle errors gracefully and return appropriate responses
3. **Service Names**: Use descriptive names that clearly indicate the service's purpose
4. **Timeout Handling**: Always handle potential timeouts in clients
5. **Resource Management**: Properly clean up service handles when shutting down

## Creating Custom Service Types

To create your own service type, create a `.srv` file in your package:

```
# In your_package/srv/CustomService.srv
# Request part
string name
int32 age
---
# Response part
bool success
string message
```

Then use it in your code by importing the generated message type.

## Summary

Services provide synchronous request-response communication in ROS 2, which is essential for operations that require a specific result. They complement topics by offering a way to request specific actions and receive responses. Understanding when to use services versus topics is crucial for designing effective robotic systems.

Now that we've covered all three fundamental ROS 2 concepts (Nodes, Topics, and Services), you have the foundation needed to build distributed robotic applications.