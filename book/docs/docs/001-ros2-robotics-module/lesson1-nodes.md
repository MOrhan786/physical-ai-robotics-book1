**Lesson 1: Understanding ROS 2 Nodes**

**What is a ROS 2 Node?**

In ROS 2, a "node" is the fundamental building block of any ROS 2 application. Think of a node as a single executable program that performs a specific task. For example, one node might control a robot's motor, another might read data from a sensor, and yet another might process images from a camera.

The beauty of ROS 2 is that these nodes are designed to be modular and communicate with each other, allowing you to break down complex robotic systems into smaller, manageable, and reusable components. Each node runs independently and can be written in different programming languages (like Python with `rclpy` or C++ with `rclcpp`).

**Key Characteristics of a Node:**

*   **Modular:** Each node does one thing well.
*   **Communicative:** Nodes communicate with each other using various mechanisms (Topics, Services, Actions – which we'll cover later).
*   **Independent:** Nodes run as separate processes. If one node crashes, it ideally shouldn't bring down the entire system.
*   **Named:** Every node has a unique name within the ROS 2 graph, making it easy to identify and interact with.

**Why use Nodes?**

*   **Reusability:** You can reuse nodes in different projects or parts of your robot.
*   **Fault Isolation:** Problems in one node are less likely to affect others.
*   **Distributed Computing:** Nodes can run on different machines across a network, allowing for powerful distributed systems.

**Creating a Simple ROS 2 Node with `rclpy` (Python)**

`rclpy` is the Python client library for ROS 2. It allows you to write ROS 2 nodes using Python. Let's create a very simple node that just initializes itself and then shuts down.

```python
# Import the rclpy library
import rclpy
# Import the Node class from rclpy.node
from rclpy.node import Node

# Define a minimal node class
class MinimalNode(Node):
    def __init__(self):
        # Call the superclass constructor to initialize the node
        # The string "minimal_node" is the name of our node
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node has been started!')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of our MinimalNode
    minimal_node = MinimalNode()

    # Keep the node alive (spinning) so it can process callbacks (not used yet, but good practice)
    # For this minimal node, we'll just spin once and then destroy it.
    rclpy.spin_once(minimal_node)

    # Destroy the node explicitly
    minimal_node.destroy_node()

    # Shut down the rclpy library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Code Explanation:**

1.  **`import rclpy`**: Imports the main ROS 2 Python client library.
2.  **`from rclpy.node import Node`**: Imports the `Node` class, which is the base class for all ROS 2 nodes in Python.
3.  **`class MinimalNode(Node):`**: We define our own node class, `MinimalNode`, which inherits from `rclpy.node.Node`.
4.  **`super().__init__('minimal_node')`**: In the constructor (`__init__`), we call the constructor of the parent `Node` class. We give our node the name `'minimal_node'`. This name is how other nodes in the ROS 2 system will identify it.
5.  **`self.get_logger().info('Minimal node has been started!')`**: Every node has a built-in logger. This line prints an informational message to the console when the node starts.
6.  **`def main(args=None):`**: This is a common entry point for Python ROS 2 programs.
7.  **`rclpy.init(args=args)`**: This function initializes the `rclpy` library. It must be called before you can create any nodes.
8.  **`minimal_node = MinimalNode()`**: We create an object (an instance) of our `MinimalNode` class.
9.  **`rclpy.spin_once(minimal_node)`**: This command allows the node to process any pending callbacks (like messages received or service requests). For this simple node, it just allows the `__init__` method's `get_logger().info` call to execute before shutdown. In more complex nodes, `rclpy.spin()` would be used to keep the node running and responsive.
10. **`minimal_node.destroy_node()`**: It's good practice to explicitly destroy the node when you're done with it to release resources.
11. **`rclpy.shutdown()`**: This shuts down the `rclpy` library.

**How to Run This Node:**

1.  Save the code above as `minimal_node.py` in a new ROS 2 package (you'll learn how to create packages later, but for now, you can just save it as a standalone Python file).
2.  Open your terminal and navigate to the directory where you saved `minimal_node.py`.
3.  Run the command: `python3 minimal_node.py`

You should see the output: `[INFO] [minimal_node]: Minimal node has been started!` followed by the program exiting.