---
title: Path Planning with Nav2
---

# 🗺️ Path Planning with Nav2

**Nav2** is the second generation of the ROS Navigation Stack. It is a powerful, production-quality system that enables a robot to move from a starting point to a goal destination safely and efficiently. While it was originally designed for wheeled mobile robots, its modular architecture allows it to be adapted for other types of robots, including humanoids.

### The Nav2 Architecture

Nav2 is not a single program but a collection of nodes, servers, and plugins that work together. The core components include:

-   **Global Planner:** Given a goal, the global planner creates a long-term plan (a path) through a known map. The default algorithm is `A*`, but it can be replaced with others.
-   **Local Planner:** The local planner is responsible for generating safe velocity commands to follow the global plan while avoiding immediate obstacles that may not be on the static map. The default is `TEB` (Timed Elastic Band) or `DWB` (Dynamic Window Approach).
-   **Controller Server:** This server hosts the local planner.
-   **Planner Server:** This server hosts the global planner.
-   **Behavior Tree:** The overall logic of Nav2 is orchestrated by a Behavior Tree (`BT`). This allows for complex and customizable navigation logic, such as recovery behaviors (e.g., "if stuck, rotate 360 degrees and try again").
-   **Costmaps:** Nav2 uses two costmaps to represent the environment: a global costmap for the long-term plan and a local costmap for short-term obstacle avoidance.

![Nav2 Architecture](https://navigation.ros.org/_images/Architectures.png)
*High-level view of the Nav2 architecture.*

### Adapting Nav2 for Humanoids

Using Nav2 with a bipedal humanoid presents unique challenges compared to a wheeled robot:
1.  **Non-holonomic Motion:** A humanoid cannot move sideways as easily as a differential drive robot. The local planner must be configured to respect these kinematic constraints.
2.  **Stability:** The controller must not generate velocity commands that would cause the robot to lose its balance. This often means the local planner's acceleration limits need to be very conservative.
3.  **Footprint:** A humanoid's footprint changes as it walks. The costmap configuration must be robust to this changing shape.

Although challenging, integrating Nav2 provides a robust foundation for high-level path planning, allowing you to focus on the lower-level problem of stable bipedal locomotion. You can send a goal to Nav2, and it will provide a smooth path for your walking controller to follow.
