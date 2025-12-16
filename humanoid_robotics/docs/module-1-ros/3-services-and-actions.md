---
title: Services and Actions
---

# 📞 Services and Actions

While Topics are ideal for continuous data streams (like sensor readings), they aren't suitable for request/reply interactions. For this, ROS provides two additional communication patterns: **Services** and **Actions**.

### Services: Synchronous Request/Reply

A **Service** is a synchronous, one-to-one communication mechanism. It's best suited for quick, remote procedure calls where a node needs a direct answer from another node.

-   A **Service Client** sends a single `Request`.
-   The client then **waits (blocks)** until it receives a `Reply`.
-   A **Service Server** receives the request, performs a task, and sends back a single `Reply`.

**When to use a Service:**
-   Querying the state of a node (e.g., "What is the robot's current position?").
-   Triggering a fast, blocking operation (e.g., "Open the gripper now.").
-   Fetching a configuration parameter.

Because the client blocks until it receives a response, services are only suitable for tasks that complete very quickly.

```python
# Conceptual Python code for a ROS 2 Service Server

# 1. Define the service interface in a .srv file
# ---
# int64 a
# int64 b
# ---
# int64 sum

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        return response
```

### Actions: Asynchronous, Long-Running Tasks

An **Action** is used for long-running, asynchronous tasks that require feedback during their execution. They are the most complex of the communication types but also the most powerful for orchestrating robot behavior.

The Action communication pattern involves three parts:
-   An **Action Client** sends a `Goal` to an Action Server (e.g., "Navigate to position X,Y").
-   The **Action Server** accepts the goal and begins executing the task. As it works, it can stream `Feedback` messages back to the client (e.g., "Current distance to goal: 5.2 meters").
-   When the task is complete, the server sends a final `Result` message.

**When to use an Action:**
-   Navigation: "Move the robot to a specific room."
-   Manipulation: "Pick up the object at this location."
-   Long computations: "Generate a complete 3D map of the environment."

Actions are non-blocking. The client can continue with other tasks while the server works on the goal, and it can even choose to cancel the goal mid-execution. This makes Actions the ideal choice for any robotic task that takes more than a fraction of a second to complete.
