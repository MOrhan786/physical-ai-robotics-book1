**Lesson 3: Understanding ROS 2 Services (Client and Server)**

**What are ROS 2 Services?**

While Topics are great for continuous, one-way data streams, sometimes you need a direct, request/response communication mechanism. This is where **Services** come in. A ROS 2 Service allows one node (the client) to send a request to another node (the server) and wait for a response. It's a synchronous, blocking call, meaning the client pauses its execution until it receives a response from the server.

*   **Service Server:** A node that offers a specific service. It waits for requests, processes them, and sends back a response.
*   **Service Client:** A node that sends a request to a service server and waits for its response.

Services are defined by a pair of message types: a request message and a response message. These are defined in `.srv` files.

**Key Characteristics of Services:**

*   **Synchronous Request/Response:** The client waits for the server's response.
*   **One-to-One Communication:** Typically, one client interacts with one server for a specific request.
*   **Defined Interface:** Services use a request/response pair defined in a `.srv` file, ensuring both client and server understand the data format.

**Why use Services?**

*   **Specific Tasks:** Ideal for tasks that require a clear trigger and a defined outcome, like commanding a robot arm to move to a specific position and waiting for confirmation.
*   **Configuration:** Changing parameters or querying state from a node.
*   **Error Handling:** The client can handle timeouts or service failures directly.

**Creating a ROS 2 Service Server and Client with `rclpy`**

To create a service, we first need to define a custom service interface. Let's assume we have a simple service called `AddTwoInts` that takes two integers and returns their sum.

**1. Define the Service Interface (`AddTwoInts.srv`)**

First, you would typically define this in a `.srv` file within a ROS 2 package. For this example, let's just imagine its definition:

```
# Request
int64 a
int64 b
---
# Response
int64 sum
```
This means the service request will contain two `int64` fields named `a` and `b`, and the response will contain one `int64` field named `sum`. When you build a ROS 2 package containing this `.srv` file, `rclpy` automatically generates Python classes for `AddTwoInts.Request` and `AddTwoInts.Response`.

---

**2. The Service Server Node (`simple_service_server.py`)**

This node will provide the `AddTwoInts` service.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # We'll use a built-in example service for simplicity

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        # Create a service.
        # Arguments: service type, service name, callback function
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server "add_two_ints" has been started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server) # Keep the node alive to listen for requests
    simple_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Server Code Explanation:**

1.  **`from example_interfaces.srv import AddTwoInts`**: For simplicity and without creating a custom package first, we use a standard `AddTwoInts` service provided by `example_interfaces`.
2.  **`self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)`**: This creates the service server.
    *   `AddTwoInts`: The service type.
    *   `'add_two_ints'`: The name of the service. Clients will use this name to call the service.
    *   `self.add_two_ints_callback`: The function that will be executed when a client makes a request.
3.  **`def add_two_ints_callback(self, request, response):`**: This is the callback function. It receives a `request` object (containing `a` and `b`) and a `response` object. We compute the sum and set `response.sum`, then return the `response` object.
4.  **`rclpy.spin(simple_service_server)`**: Keeps the server node running, waiting for incoming requests.

---

**3. The Service Client Node (`simple_service_client.py`)**

This node will request the `AddTwoInts` service.

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Same service type as the server

class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__('simple_service_client')
        # Create a client for the AddTwoInts service
        # Arguments: service type, service name
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait until the service is available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request() # Create a request object

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req) # Send the request asynchronously
        self.get_logger().info(f'Requesting to add {self.req.a} and {self.req.b}')
        return self.future

def main(args=None):
    rclpy.init(args=args)
    simple_service_client = SimpleServiceClient()

    if len(sys.argv) != 3:
        simple_service_client.get_logger().info('Usage: ros2 run <package_name> simple_service_client A B')
        simple_service_client.destroy_node()
        rclpy.shutdown()
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])
    future = simple_service_client.send_request(a, b)

    # Spin until the future is complete (response received)
    rclpy.spin_until_future_complete(simple_service_client, future)

    if future.result() is not None:
        simple_service_client.get_logger().info(
            f'Result of add_two_ints: for {a} + {b} = {future.result().sum}'
        )
    else:
        simple_service_client.get_logger().error('Service call failed %r' % (future.exception(),))

    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client Code Explanation:**

1.  **`self.create_client(AddTwoInts, 'add_two_ints')`**: This creates the service client, specifying the service type and name.
2.  **`while not self.cli.wait_for_service(...)`**: This loop ensures the client waits until the service server is actually running and available.
3.  **`self.req = AddTwoInts.Request()`**: We create an instance of the request message type.
4.  **`self.future = self.cli.call_async(self.req)`**: This sends the request to the server. `call_async` is used because in Python, waiting for a service response would block the entire node. `call_async` allows the node to continue spinning while waiting for the response. The response will be stored in the `future` object.
5.  **`rclpy.spin_until_future_complete(simple_service_client, future)`**: This function specifically waits for the `future` object to complete (i.e., the response has been received) while still allowing the node to process other callbacks in the meantime.
6.  **`if future.result() is not None:`**: Once the future is complete, `future.result()` contains the `AddTwoInts.Response` object. We check if it's not `None` (meaning the call was successful) and print the sum.

---

**How to Run These Nodes:**

1.  Save the service server code as `simple_service_server.py` and the service client code as `simple_service_client.py` in the same directory.
2.  Open **two separate terminals**.
3.  In the first terminal, run the service server:
    ```bash
    python3 simple_service_server.py
    ```
4.  In the second terminal, run the service client, providing two integers as arguments:
    ```bash
    python3 simple_service_client.py 5 3
    ```
    You can try different numbers.

You should see the server terminal log the incoming request and the outgoing response, and the client terminal log the request sent and the result received from the server (e.g., `Result of add_two_ints: for 5 + 3 = 8`).

---

This covers the third part of Module 1. I will now write this content to `specs/001-ros2-robotics-module/lesson3-services.md`.