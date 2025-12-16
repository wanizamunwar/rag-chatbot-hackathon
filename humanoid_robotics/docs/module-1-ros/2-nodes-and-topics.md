---
title: Nodes and Topics
---

# 🧩 Nodes and Topics

At the heart of ROS is a communication model that allows for complex systems to be built from simple, decoupled components. The two most fundamental of these components are **Nodes** and **Topics**.

### Nodes: The Building Blocks

A **Node** is the smallest unit of computation in ROS. Think of a single node as a small program that performs one specific task. For example, a humanoid robot's software might be split into several nodes:

-   `camera_driver_node`: Manages the connection to a camera and publishes image data.
-   `object_detection_node`: Subscribes to image data and publishes the locations of detected objects.
-   `navigation_node`: Determines the path for the robot to follow.
-   `motor_controller_node`: Receives motor commands and controls the robot's joints.

This modularity means you can develop, test, and replace individual parts of the system without affecting others.

![Nodes and Topics Diagram](https://raw.githubusercontent.com/osrf/ros_tutorials/rolling/ros_tutorials/rqt_graph/images/rqt_graph.png)
*A visual representation of nodes (ovals) and topics (squares) communicating.*

### Topics: The Communication Buses

Nodes need a way to exchange data. **Topics** are the named buses that fulfill this role. They work on a **publish/subscribe** model:

-   A node **publishes** messages of a specific type to a topic. It doesn't know or care who is receiving the data.
-   Other nodes can **subscribe** to that same topic to receive the messages. They don't know or care who is sending the data.

This completely decouples the producer of information from the consumer. A topic has a defined **message type** (e.g., `String`, `Int32`, `sensor_msgs/Image`), which acts as a contract, ensuring that all nodes communicating on that topic are speaking the same language.

### Example: A Simple "Chatter" System

Let's imagine a simple system with two nodes:
1.  A `talker` node that publishes `String` messages to a topic named `/chatter`.
2.  A `listener` node that subscribes to the `/chatter` topic and prints any messages it receives.

```bash
# Terminal 1: Run the talker node
ros2 run demo_nodes_py talker

# Terminal 2: Run the listener node
ros2 run demo_nodes_py listener
```

In Terminal 1, you would see:
```
[INFO] [talker]: Publishing: "Hello World: 1"
[INFO] [talker]: Publishing: "Hello World: 2"
...
```

In Terminal 2, you would see:
```
[INFO] [listener]: I heard: "Hello World: 1"
[INFO] [listener]: I heard: "Hello World: 2"
...
```

The `talker` and `listener` nodes are completely independent. You could stop the `listener`, and the `talker` would continue publishing without an issue. You could also add five more `listener` nodes, and they would all receive the same messages. This powerful and flexible model is the foundation of all ROS communication.
