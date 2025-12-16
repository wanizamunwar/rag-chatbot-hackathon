---
title: Building Worlds in Gazebo
---

# 🏗️ Building Worlds in Gazebo

**Gazebo** is a mature, open-source 3D robotics simulator that is tightly integrated with the ROS ecosystem. Its primary strength lies in its high-fidelity physics simulation, making it an excellent tool for testing dynamics, control algorithms, and sensor interactions.

### The Gazebo Architecture

Gazebo runs as a separate server (`gzserver`) and client (`gzclient`) architecture.
-   `gzserver`: Runs the physics loop, generates sensor data, and handles all the core computation. You can run `gzserver` in a headless mode on a server.
-   `gzclient`: Provides the 3D graphical interface to visualize the world and interact with the simulation.

This separation allows for efficient, distributed simulation.

### Creating Worlds with SDF

While you can use the Gazebo GUI to build simple worlds, the standard and most powerful way to define a simulated environment is with the **Simulation Description Format (SDF)**. SDF is an XML format, similar to URDF, but designed to describe everything about a simulation, not just a single robot.

An SDF file can specify:
-   **Models:** Robots, furniture, and other objects in the environment. These can be defined directly or imported from other files (like URDFs).
-   **Physics:** Global physics properties like gravity.
-   **Lighting:** Light sources, shadows, and atmospheric effects.
-   **Scenery:** Sky, ground planes, and other visual elements.

Here's a snippet of an SDF file defining a simple ground plane and a light source:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

  </world>
</sdf>
```

### Gazebo and ROS Integration

The true power of Gazebo is realized through its ROS integration. Gazebo plugins allow you to:
-   Spawn URDF models into the simulation directly from ROS nodes.
-   Publish sensor data from simulated sensors (like cameras and LiDAR) directly to ROS topics.
-   Control joints and motors by subscribing to ROS topics or by using ROS services and actions.

This deep integration allows you to run the exact same ROS software stack in simulation as you do on the real robot, which is a critical step in bridging the "reality gap".
