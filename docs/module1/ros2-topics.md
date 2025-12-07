# Chapter 2: Communicating with ROS 2 Topics

Nodes communicate with each other by publishing messages to **topics**. Topics are named buses over which nodes exchange messages. This chapter explains how to use topics to send and receive data.

## What is a Topic?

A topic is a named channel that nodes use to communicate with each other. Nodes can **publish** messages to a topic, and other nodes can **subscribe** to that topic to receive the messages. This publish/subscribe model is a flexible and powerful way to decouple nodes from each other. Think of it like a mailing list: a publisher sends an email to the list, and everyone who has subscribed to that list receives a copy.

A key feature of topics is that they create an anonymous, many-to-many communication channel. A publisher doesn't know which nodes (or how many) are subscribed, and a subscriber doesn't know where the messages are coming from. This decoupling is essential for building large, modular robotics systems.

## Topic Naming Conventions

Effective topic naming is crucial for maintaining a clean and understandable ROS 2 system. Well-named topics make it easier to debug, inspect, and integrate new components. Here are some established best practices:

- **Use Namespaces**: Group related topics under a common namespace. For example, a camera driver might publish images to `/camera/image_raw` and camera info to `/camera/camera_info`. This avoids naming conflicts and clarifies the source of the data.
- **Lowercase and Underscores**: Topic names should be in `lowercase_with_underscores`.
- **Clarity and Specificity**: Names should be specific enough to be unambiguous. For example, `/robot/left_arm/joint_states` is much clearer than just `/joint_states`.
- **Avoid Redundancy**: Don't include the message type in the topic name (e.g., use `/robot/odometry` instead of `/robot/odometry_msg`).

## Quality of Service (QoS)

ROS 2 introduces Quality of Service (QoS) policies that allow you to configure how topics handle message delivery. These settings are critical for managing network traffic and ensuring reliable communication, especially in lossy wireless environments.

Key QoS settings include:

- **History**:
    - `KEEP_LAST`: Caches up to a specified number of messages (defined by `depth`).
    - `KEEP_ALL`: Caches all messages, but can consume significant memory.
- **Depth**: When `History` is set to `KEEP_LAST`, `depth` specifies the number of messages to cache for late-joining subscribers.
- **Reliability**:
    - `RELIABLE`: Guarantees delivery, but with higher overhead. It will retry sending a message until the subscriber acknowledges it.
    - `BEST_EFFORT`: Delivers messages without acknowledgement, which is faster but less reliable. This is suitable for high-frequency data where missing a few messages is acceptable (e.g., sensor readings).
- **Durability**:
    - `VOLATILE`: Subscribers will only receive messages that are published after they have subscribed.
    - `TRANSIENT_LOCAL`: New subscribers will receive the last published message(s) on the topic, even if they were published before the subscriber joined. This is useful for topics that publish static data, like a map.

You can specify QoS profiles when creating publishers and subscribers to fine-tune communication for different use cases.

## Using Topics

Here is an example of a Python node that publishes a message to a topic:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        
        # Define a QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher_ = self.create_publisher(String, 'my_topic', qos_profile)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

And here is a node that subscribes to that topic with a matching QoS profile:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        
        # Define a QoS profile that is compatible with the publisher
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

When you run these two nodes, you will see the publisher node sending messages and the subscriber node receiving them. This is the fundamental way that nodes communicate in ROS 2.

## Command-Line Tools for Topics

ROS 2 provides a set of command-line tools to inspect and interact with topics directly from the terminal. These are invaluable for debugging.

- **`ros2 topic list`**: Lists all active topics in the system.
- **`ros2 topic info <topic_name>`**: Shows the message type, number of publishers, and number of subscribers for a given topic.
- **`ros2 topic echo <topic_name>`**: Prints the messages being published to a topic in real-time.
- **`ros2 topic pub <topic_name> <message_type> '<data>'`**: Publishes a single message to a topic from the command line. The data must be in YAML format.

For example, to publish a message to `my_topic`:
```bash
ros2 topic pub /my_topic std_msgs/msg/String "data: 'Hello from the command line'"
```

## Sources & References

- [ROS 2 Documentation - Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [ROS 2 Documentation - Quality of Service](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)