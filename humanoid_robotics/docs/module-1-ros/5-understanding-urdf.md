---
title: Understanding URDF
---

# 🛠️ Understanding URDF

The **Unified Robot Description Format (URDF)** is an essential XML specification used in ROS to describe the physical structure of a robot. A URDF file is the blueprint that tells the ROS ecosystem what your robot looks like, how its parts are connected, and how they move.

A URDF file is composed of two primary components: **links** and **joints**.

### Links: The Bones of the Robot

A **`<link>`** element represents a rigid part of the robot's body. Each link has a name and can contain several properties, but the most important are `<visual>` and `<collision>`.

-   **`<visual>`**: This tag describes what the link looks like. It defines the 3D geometry, which is usually a mesh file (like `.stl` or `.dae`), the origin (position and orientation) of the mesh relative to the link's frame, and its material (color and texture). This is what you see in a simulator like RViz.
-   **`<collision>`**: This tag describes the link's collision geometry. This is what the physics engine uses to calculate collisions. It is often a simpler shape (like a cylinder or box) than the visual geometry to save computation, but it should closely approximate the link's physical bounds.

```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue"/>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
</link>
```

### Joints: The Muscles and Connections

A **`<joint>`** element describes the kinematics and dynamics of the connection between two links. Each joint connects one **parent** link to one **child** link.

Key properties of a joint include:
-   **`name`**: A unique name for the joint.
-   **`type`**: The most important property, defining how the child link can move relative to the parent. Common types are:
    -   `revolute`: A hinge joint that rotates around a single axis (e.g., an elbow).
    -   `continuous`: A revolute joint with no angle limits.
    -   `prismatic`: A sliding joint that moves along an axis (e.g., a piston).
    -   `fixed`: A rigid connection with no movement.
-   **`parent`** and **`child`**: The names of the two links being connected.
-   **`axis`**: The axis of rotation or translation for `revolute` and `prismatic` joints.
-   **`limit`**: For `revolute` and `prismatic` joints, this specifies the upper and lower limits of motion and the maximum velocity and effort.

```xml
<joint name="base_to_arm" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>
```

By connecting a series of links and joints, you can describe the entire kinematic chain of a robot, from its base to its end-effector. This URDF model is fundamental for simulation, path planning, and visualization in ROS.
