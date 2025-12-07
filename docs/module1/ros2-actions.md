# Chapter 4: Handling Long-Running Tasks with ROS 2 Actions

For tasks that take a long time to complete and need to provide feedback along the way, neither topics nor services are a perfect fit. ROS 2 provides another communication pattern called **actions** for exactly these scenarios. Actions are the backbone of complex robotics behaviors like navigation, manipulation, and executing multi-step sequences.

## What is an Action?

An action is designed for long-running, preemptible tasks. It consists of three parts, defined in a `.action` file:
1.  **Goal**: The client sends a goal to the action server, specifying the desired outcome (e.g., "move the robot arm to joint angles [1.5, 0.8, -1.2]").
2.  **Feedback**: While processing the goal, the server provides regular updates on its progress (e.g., "current joint angles are [1.2, 0.6, -1.0]").
3.  **Result**: Once the task is complete, the server sends a final result (e.g., "successfully reached target joint angles").

This structure allows for robust handling of complex behaviors. Think of it like ordering a pizza: you place an order (the goal), you can call to check on the status (the feedback), and you eventually receive the pizza (the result). A key feature of actions is that the client can request to **cancel** a goal before it has completed, and the server can accept or reject the cancellation.

## Action Definition Files (.action)

Custom actions are defined in `.action` files, which are split into three sections separated by `---` lines.

1.  The **goal** definition comes first.
2.  The **result** definition comes second.
3.  The **feedback** definition comes third.

For example, here is a simplified `MoveArm.action` file:

```c
# Goal: target joint angles for the arm
float64[] target_angles
---
# Result: boolean indicating if the arm reached the target
bool success
---
# Feedback: the current joint angles of the arm during the move
float64[] current_angles
```

These files are stored in an `action/` directory within your package and must be registered in your `CMakeLists.txt` and `package.xml` to be built.

## Using Actions

Here is a more complete example of an action server that simulates a long-running task and handles goal cancellation.

```python
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """A new goal has been accepted and is ready to be executed."""
        self.get_logger().info('Goal accepted, starting execution...')
        # We assign a thread to this goal to avoid blocking the executor
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Executes the action."""
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info(f'Returning result: {result.sequence}')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

And here is the corresponding action client, with added logic for sending a cancellation request.

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == rclpy.action.GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Result: {result.sequence}')
        else:
            self.get_logger().info(f'Goal failed with status: {status}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.partial_sequence}')
    
    def cancel_goal(self):
        self.get_logger().info('Canceling goal')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    # You could add logic here to cancel the goal after a few seconds
    # For example: time.sleep(3); action_client.cancel_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## Command-Line Tools for Actions

ROS 2 provides tools for interacting with actions from the command line.

- **`ros2 action list`**: Lists all available action servers.
- **`ros2 action info <action_name>`**: Shows the action type and number of clients/servers.
- **`ros2 action send_goal <action_name> <action_type> '<goal>'`**: Sends a goal to an action server. You can also request feedback with the `--feedback` flag.

For example, to send a goal to the Fibonacci action server:
```bash
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}" --feedback
```

## Sources & References

- [ROS 2 Documentation - Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- [ROS 2 Documentation - Custom Action Files](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-Interface-Python.html)