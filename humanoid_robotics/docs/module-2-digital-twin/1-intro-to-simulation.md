---
title: Introduction to Simulation
---

# 🌍 Introduction to Simulation

In robotics, a **digital twin** is a virtual model of a physical robot that is so accurate it can be used to test and validate software before it's deployed on real hardware. This "sim-to-real" workflow is a cornerstone of modern robotics development.

### Why Simulate?

Developing directly on physical hardware is often slow, expensive, and risky. Simulation offers a powerful alternative with several key advantages:

-   **🚀 Rapid Prototyping:** You can test new algorithms and ideas in minutes in a simulator, whereas setting up a hardware test could take hours.
-   **💰 Cost-Effectiveness:** Simulators are free. Physical robots and sensors are not. Crashing a virtual robot costs nothing, but crashing a real one can be a very expensive mistake.
-   **🛡️ Safety:** You can safely test dangerous scenarios, such as robot-human collaboration or recovery from critical failures, without any real-world risk.
-   **🤖 Parallelization:** In the cloud, you can run thousands of simulations in parallel to train reinforcement learning agents or perform large-scale testing, a feat impossible with a limited number of physical robots.

### The Challenge: The "Reality Gap"

The biggest challenge in simulation is the **"reality gap"**: the difference between how a robot behaves in simulation versus how it behaves in the real world. This gap can be caused by many factors:
-   Inaccurate physics parameters (friction, mass, inertia).
-   Sensor noise models that don't match reality.
-   Differences in timing between the simulated and real software.

A significant part of robotics engineering involves minimizing this gap. Techniques like **domain randomization**, where the simulator's parameters are varied during training, help create more robust models that transfer better to the real world.

In this module, we will explore two powerful simulators that help us bridge this gap: **Gazebo** for robust physics and **Unity** for high-fidelity rendering.
