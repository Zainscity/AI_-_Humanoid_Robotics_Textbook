# Chapter 1: Understanding ROS 2 Nodes

A ROS 2 system is a distributed network of processes called **nodes**. Each node is a fundamental unit of computation and should be responsible for a single, modular purpose (e.g., one node for controlling wheel motors, one for processing camera images, and another for planning paths). This chapter explains what nodes are and how to create and manage them.

## What is a Node?

A node is a standalone executable that performs a specific task within the ROS 2 graph. Nodes communicate with each other using ROS 2 primitives like topics, services, and actions. This modular design allows for high levels of fault tolerance, scalability, and reusability. If a node crashes, the rest of the system can continue to operate.

## Creating a Node

Creating a node in ROS 2 is straightforward. The `rclpy` library provides a `Node` class that you can inherit from.

Here is a basic example of a Python node:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # The node is initialized with the name 'my_node'
        super().__init__('my_node')
        self.get_logger().info('Hello from my_node! This node is now active.')
        
        # You can create timers, publishers, subscribers, etc. here
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Timer ticked')

def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)
    
    # Create an instance of the node
    node = MyNode()
    
    try:
        # Spin the node to keep it alive and allow callbacks to be processed
        rclpy.spin(node)
    except KeyboardInterrupt:
        # This allows you to shut down the node cleanly with Ctrl+C
        pass
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        # Shutdown the ROS 2 client library
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This node, when run, will print a message to the console once per second. This demonstrates the fundamental structure of a ROS 2 node, including initialization, logging, and the event loop (`spin`).

## Node Composition

For performance-critical applications, running many nodes as separate processes can introduce latency due to inter-process communication (IPC). ROS 2 offers **composition**, which allows you to compile multiple nodes into a single process. This enables nodes to communicate by passing pointers to messages rather than serializing and deserializing them, significantly reducing overhead.

You can create a **component** (a special type of node) and load it into a generic container process at runtime. This provides the flexibility of separate processes with the performance of a single, monolithic one.

## Launch Files

In a real robotics system, you'll often need to run and configure dozens of nodes simultaneously. ROS 2 **launch files** are Python scripts that allow you to automate this process. With a launch file, you can:

- Start multiple nodes with a single command.
- Set parameters for each node.
- Remap topic, service, and action names.
- Group nodes into namespaces.
- Conditionally launch nodes based on arguments.

Here is a simple example of a launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node_executable',
            name='custom_name_for_my_node',
            parameters=[{'my_param': 'some_value'}]
        ),
        Node(
            package='another_package',
            executable='another_node',
            namespace='robot1'
        )
    ])
```
To run this, you would use the command: `ros2 launch my_package my_launch_file.py`.

## Command-Line Tools for Nodes

ROS 2 provides tools for interacting with nodes from the command line.

- **`ros2 run <package_name> <executable_name>`**: The most basic command to run a single node.
- **`ros2 node list`**: Lists all running nodes in the ROS 2 graph.
- **`ros2 node info <node_name>`**: Displays detailed information about a specific node, including its publications, subscriptions, services, and actions. This is extremely useful for debugging.

## Sources & References

- [ROS 2 Documentation - Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [ROS 2 Documentation - Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [ROS 2 Documentation - Composition](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html)