---
title: Python with rclpy
---

# 🐍 Python with rclpy

`rclpy` (ROS Client Library for Python) is your gateway to the ROS 2 ecosystem when working with Python. It provides a clean, idiomatic Python interface to all the core ROS concepts, allowing you to create nodes, publishers, subscribers, and more.

### Setting up a Python ROS 2 Package

A ROS 2 package is simply a directory with a `package.xml` file and a `setup.py` file. The `package.xml` file contains metadata about the package, while the `setup.py` file is used to install the package and its executables.

A typical structure looks like this:
```
my_python_pkg/
├── package.xml
├── setup.py
├── setup.cfg
└── my_python_pkg/
    ├── __init__.py
    └── my_node.py
```

### Writing a "Talker" Node

Let's look at the "talker" node from our earlier example in more detail. This node simply publishes a "Hello World" message with a counter to the `/chatter` topic every half a second.

**File: `my_python_pkg/my_node.py`**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    """
    A simple ROS 2 node that publishes a string message periodically.
    """
    def __init__(self):
        super().__init__('talker_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.counter = 0
        self.get_logger().info('Talker node has been started and is publishing.')

    def timer_callback(self):
        """
        Called every `timer_period` seconds.
        Constructs and publishes a message.
        """
        msg = String()
        msg.data = f'Hello, ROS 2 World: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create an instance of the node
    talker_node = TalkerNode()
    
    # "Spin" the node, which allows its callbacks to be executed
    rclpy.spin(talker_node)
    
    # Clean up and shutdown
    talker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Key Concepts in the Code:
1.  **`rclpy.init()`**: Initializes the ROS 2 communication. This must be the first line of any `rclpy` application.
2.  **`Node`**: We create a class that inherits from `rclpy.node.Node`.
3.  **`create_publisher()`**: Creates a publisher object that can send messages on a given topic. We specify the message type (`String`), topic name (`chatter`), and queue size (`10`).
4.  **`create_timer()`**: Creates a timer that calls a function (`timer_callback`) at a regular interval.
5.  **`rclpy.spin()`**: This is the main event loop. It keeps the node running and responsive to events like timer callbacks.
6.  **`rclpy.shutdown()`**: Cleans up the ROS 2 resources.
