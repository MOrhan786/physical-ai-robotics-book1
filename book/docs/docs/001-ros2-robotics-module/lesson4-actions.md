---
id: lesson4-actions
title: Understanding ROS 2 Actions
sidebar_label: ROS 2 Actions
---

## What are ROS 2 Actions?

In ROS 2, **Actions** are a higher-level communication mechanism designed for long-running, asynchronous tasks that require feedback and the ability to be preempted (cancelled). Think of an action as a goal-oriented interaction where a client requests a server to perform a task, and the server continuously provides feedback on the task's progress until it's completed or cancelled.

Actions combine aspects of both Topics and Services:
*   **Like Services:** An action client sends a request (goal) to an action server, and the server sends a response (result).
*   **Like Topics:** An action server continuously sends feedback to the client about the goal's progress.

This makes Actions ideal for tasks like:
*   **Navigation:** "Go to X, Y coordinates," with feedback on current position and remaining distance.
*   **Manipulation:** "Pick up object A," with feedback on gripper state and object stability.
*   **Complex Processes:** Any task that takes time and benefits from progress updates.

## Components of an Action:

1.  **Goal:** The request sent from the client to the server (e.g., "move 10 meters forward").
2.  **Feedback:** Continuous updates sent from the server to the client about the progress of the goal (e.g., "robot has moved 5 meters").
3.  **Result:** The final outcome sent from the server to the client once the goal is completed (e.g., "goal reached successfully").

Actions are defined by a `.action` file, which specifies the structure for the goal, result, and feedback messages.

### Example `Fibonacci.action` definition:
```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```
This defines an action where a client requests a Fibonacci sequence of a certain `order`. The server provides `partial_sequence` as feedback during computation and the full `sequence` as the final result.

## Creating a ROS 2 Action Server and Client with `rclpy`

To illustrate, let's create a simple action that computes a Fibonacci sequence.

### 1. The Action Server Node (`simple_action_server.py`)

This node will receive an `order` (number) and compute the Fibonacci sequence up to that order, sending partial sequences as feedback and the final sequence as the result.

```python
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from example_interfaces.action import Fibonacci # We use the built-in Fibonacci action


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info('Fibonacci Action Server has been started.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal: {0}'.format(goal_handle.request.order))

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Start executing the action
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info('Goal succeeded, result: {0}'.format(result.sequence))
        return result


def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```

**Action Server Code Explanation:**

1.  **`from rclpy.action import ActionServer`**: Imports the `ActionServer` class.
2.  **`from example_interfaces.action import Fibonacci`**: We use the standard `Fibonacci` action type for simplicity.
3.  **`ActionServer(...)`**: Creates the action server.
    *   `self`: The node instance.
    *   `Fibonacci`: The action type.
    *   `'fibonacci'`: The name of the action.
    *   `self.execute_callback`: The function that will be called when a client sends a goal.
4.  **`execute_callback(self, goal_handle)`**: This method handles the actual goal execution.
    *   `goal_handle.request.order`: Accesses the goal request (e.g., the desired Fibonacci `order`).
    *   `goal_handle.publish_feedback(feedback_msg)`: Sends progress updates to the client.
    *   `goal_handle.is_cancel_requested`: Allows checking if the client has requested to cancel the goal.
    *   `goal_handle.succeed()` / `goal_handle.canceled()`: Marks the goal as successful or cancelled.
    *   Returns `Fibonacci.Result()`: Sends the final result to the client.

### 2. The Action Client Node (`simple_action_client.py`)

This node will send a goal to the Fibonacci action server and process its feedback and final result.

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from example_interfaces.action import Fibonacci # Same action type as the server


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self.get_logger().info('Fibonacci Action Client has been started.')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10) # Request Fibonacci sequence up to order 10
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

**Action Client Code Explanation:**

1.  **`from rclpy.action import ActionClient`**: Imports the `ActionClient` class.
2.  **`ActionClient(...)`**: Creates the action client.
    *   `self`: The node instance.
    *   `Fibonacci`: The action type.
    *   `'fibonacci'`: The name of the action to connect to.
3.  **`send_goal(self, order)`**: Prepares and sends the goal.
    *   `self._action_client.wait_for_server()`: Blocks until the action server is available.
    *   `self._action_client.send_goal_async(...)`: Sends the goal. Crucially, it's `_async` because waiting for the result would block the node. We also provide a `feedback_callback` here.
    *   `_send_goal_future.add_done_callback(...)`: Registers a callback to be executed once the server *accepts* or *re`_send_goal_future.add_done_callback(...)`: Registers a callback to be executed once the server *accepts* or *rejects* the goal.
4.  **`goal_response_callback(self, future)`**: Handles the server's response to the goal request (acceptance or rejection). If accepted, it then requests the final result using `get_result_async()`.
5.  **`get_result_callback(self, future)`**: Handles the final result from the server.
6.  **`feedback_callback(self, feedback_msg)`**: This function is called every time the server publishes feedback.

## How to Run These Nodes:

1.  Save the action server code as `simple_action_server.py` and the action client code as `simple_action_client.py` in the same directory.
2.  Open **two separate terminals**.
3.  In the first terminal, run the action server:
    ```bash
    python3 simple_action_server.py
    ```
4.  In the second terminal, run the action client:
    ```bash
    python3 simple_action_client.py
    ```
You will see the client send a goal, the server execute it step-by-step providing feedback, and finally, the client receiving the complete result.