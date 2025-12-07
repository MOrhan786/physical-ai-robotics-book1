**Lesson 2: Understanding ROS 2 Topics (Publishers and Subscribers)**

**What are ROS 2 Topics?**

While nodes are the fundamental units, **Topics** are the most common way for nodes to exchange data in a ROS 2 system. Think of a Topic as a named channel through which nodes can send (publish) and receive (subscribe to) messages. It's a highly efficient, one-way communication mechanism based on a publish/subscribe pattern.

*   **Publishers:** A node that sends messages to a topic is called a publisher. It "advertises" its intention to publish on a specific topic.
*   **Subscribers:** A node that receives messages from a topic is called a subscriber. It "listens" for messages on a specific topic.

Multiple publishers can send messages to the same topic, and multiple subscribers can receive messages from the same topic. Subscribers don't know who the publishers are, and publishers don't know who the subscribers are. They only know about the topic. This decoupling is a powerful feature of ROS 2.

**Key Characteristics of Topics:**

*   **Anonymous Publish/Subscribe:** Senders and receivers are decoupled.
*   **One-to-Many, Many-to-One, Many-to-Many:** Flexible communication patterns.
*   **Asynchronous:** Messages are sent and received asynchronously.
*   **Data Types (Interfaces):** Every message sent over a topic has a defined type (e.g., `std_msgs/String`, `geometry_msgs/Twist`). This ensures that publishers and subscribers agree on the format of the data being exchanged.

**Why use Topics?**

*   **Real-time Data Streams:** Ideal for continuous data flows like sensor readings (camera images, lidar scans), motor commands, or robot odometry.
*   **Decoupling:** Nodes can be developed and tested independently.
*   **Scalability:** Easily add more publishers or subscribers without changing existing code.

**Creating a ROS 2 Publisher and Subscriber with `rclpy`**

Let's create two Python nodes using `rclpy`: a publisher that sends "Hello, ROS 2!" messages and a subscriber that receives and prints them.

---

**1. The Publisher Node (`simple_publisher.py`)**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the standard String message type

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        # Create a publisher.
        # Arguments: message type, topic name, queue size
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.i = 0
        # Create a timer to publish messages every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Simple Publisher node has started.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, ROS 2! ({self.i})'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher) # Keep the node alive to publish messages
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Publisher Code Explanation:**

1.  **`from std_msgs.msg import String`**: We import the `String` message type from the `std_msgs` package. This is a common ROS 2 message type for plain text.
2.  **`self.create_publisher(String, 'chatter', 10)`**: This line creates the publisher.
    *   `String`: The message type it will publish.
    *   `'chatter'`: The name of the topic. Both publisher and subscriber must use the same topic name.
    *   `10`: The queue size. This limits the number of messages that can be queued up if the subscriber is slow.
3.  **`self.create_timer(0.5, self.timer_callback)`**: This creates a timer that calls the `timer_callback` method every 0.5 seconds.
4.  **`self.publisher_.publish(msg)`**: Inside `timer_callback`, we create a `String` message, set its `data` field, and then publish it using the publisher object.
5.  **`rclpy.spin(simple_publisher)`**: This keeps the node running indefinitely, allowing the timer to continuously call `timer_callback` and publish messages.

---

**2. The Subscriber Node (`simple_subscriber.py`)**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the standard String message type

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        # Create a subscriber.
        # Arguments: message type, topic name, callback function, queue size
        self.subscriber_ = self.create_subscription(String, 'chatter', self.listener_callback, 10)
        self.get_logger().info('Simple Subscriber node has started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber) # Keep the node alive to listen for messages
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscriber Code Explanation:**

1.  **`self.create_subscription(String, 'chatter', self.listener_callback, 10)`**: This line creates the subscriber.
    *   `String`: The message type it expects to receive.
    *   `'chatter'`: The name of the topic to subscribe to.
    *   `self.listener_callback`: The function that will be called every time a new message is received on the topic.
    *   `10`: The queue size, similar to the publisher.
2.  **`def listener_callback(self, msg):`**: This is the callback function. It receives the incoming message as an argument (`msg`) and then prints its data.
3.  **`rclpy.spin(simple_subscriber)`**: This keeps the node running indefinitely, allowing it to continuously listen for and process incoming messages.

---

**How to Run These Nodes:**

1.  Save the publisher code as `simple_publisher.py` and the subscriber code as `simple_subscriber.py` in the same directory.
2.  Open **two separate terminals**.
3.  In the first terminal, run the publisher:
    ```bash
    python3 simple_publisher.py
    ```
4.  In the second terminal, run the subscriber:
    ```bash
    python3 simple_subscriber.py
    ```

You should see the publisher terminal printing "Publishing:..." messages and the subscriber terminal printing "I heard:..." messages, demonstrating communication between the two nodes via the `chatter` topic.

---

This covers the second part of Module 1. I will now write this content to `specs/001-ros2-robotics-module/lesson2-topics.md`.