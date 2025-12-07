# Chapter 3: Using ROS 2 Services for Request/Response

While topics are great for continuous data streams, sometimes you need a direct request/response interaction between nodes. This is where **services** come in. This chapter will introduce you to the service communication pattern in ROS 2.

## What is a Service?

A service is a type of communication that allows one node (the **client**) to send a request to another node (the **server**) and wait for a response. This is a synchronous, one-to-one communication model, unlike the asynchronous, many-to-many model of topics. Services are ideal for tasks that have a clear beginning and end, and where a direct confirmation of success is needed, such as triggering a computation or changing a setting.

A service is defined by a `.srv` file, which specifies the structure of the request and the response.

## Service Definition Files (.srv)

To create a custom service, you must define a `.srv` file. This file is split into two parts—the request and the response—separated by a `---` line.

For example, let's create a `SetLed.srv` file to control an LED on a robot.

```c
# Request: which LED to set (e.g., 0 for the first LED) and its state (on/off)
int64 led_number
bool state
---
# Response: a boolean indicating if the operation was successful
bool success
```

This file would be placed in a `srv/` directory within your package. After defining the service, you need to update your `CMakeLists.txt` and `package.xml` to build it, similar to how custom messages are handled.

## Using Services

Here is an example of a Python node that acts as a service server. It uses the standard `AddTwoInts` service for simplicity.

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server_node')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server is ready to receive requests.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}')
        self.get_logger().info(f'Sending back response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceServerNode()
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

### Synchronous Client

The client node sends a request and blocks until it receives a response. This is simple, but it can freeze your node if the service takes a long time to respond.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client_node')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        # This call blocks until the service responds
        self.future = self.client.call(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    node = ServiceClientNode()
    response = node.send_request(2, 3)
    if response is not None:
        node.get_logger().info(
            f'Result of add_two_ints: for {2} + {3} = {response.sum}')
    else:
        node.get_logger().error('Service call failed')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Asynchronous Client

A better approach for many applications is to use an asynchronous client. This allows the node to continue executing other tasks while waiting for the service response. The response is handled in a callback function once it arrives.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AsyncServiceClientNode(Node):
    def __init__(self):
        super().__init__('async_service_client_node')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        # call_async does not block. It returns a future object.
        self.future = self.client.call_async(self.req)
        # We can add a callback to be executed when the future is complete
        self.future.add_done_callback(self.future_callback)
        self.get_logger().info('Request sent, node can do other work...')

    def future_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {self.req.a} + {self.req.b} = {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AsyncServiceClientNode()
    node.send_request(5, 10)
    
    # Keep the node alive to receive the response
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            break
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Command-Line Tools for Services

ROS 2 includes command-line tools for interacting with services.

- **`ros2 service list`**: Lists all available services.
- **`ros2 service type <service_name>`**: Displays the service type (e.g., `example_interfaces/srv/AddTwoInts`).
- **`ros2 service find <service_type>`**: Finds all services of a given type.
- **`ros2 service call <service_name> <service_type> '<request>'`**: Calls a service from the command line using a YAML-formatted request.

For example, to call the `add_two_ints` service:
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"
```

## Sources & References

- [ROS 2 Documentation - Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [ROS 2 Documentation - Custom Service Files](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-Interface-Python.html)
