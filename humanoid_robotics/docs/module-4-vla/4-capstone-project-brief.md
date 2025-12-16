---
title: Capstone Project Brief
---

# 🏆 Capstone Project: The Autonomous Humanoid

This capstone project is the culmination of all the skills you have developed throughout this book. Your mission is to design, build, and demonstrate a simulated autonomous humanoid robot that can understand a high-level voice command and execute a multi-step "pick and place" task.

### Project Objective

In a simulated environment (using either Gazebo or Isaac Sim), your robot must successfully perform the following sequence:

1.  **Receive a Command:** The user will give a voice command, such as, "Please fetch the green can from the kitchen table and place it on the counter."
2.  **Perceive the World:** The robot must use its simulated camera to identify the key objects and locations mentioned in the command (e.g., "kitchen table," "green can," "counter").
3.  **Formulate a Plan:** A central "Orchestrator" node, likely using an LLM, will break down the command into a sequence of high-level actions.
    -   Example Plan: `[navigate_to('kitchen_table'), pick_up('green_can'), navigate_to('counter'), place('green_can', 'counter')]`
4.  **Navigate and Execute:** The robot must use the Nav2 stack to navigate to the first location.
5.  **Manipulate:** Upon arrival, it must use its perception system to locate the target object, and then use its arm and gripper to pick it up.
6.  **Complete the Task:** The robot will then navigate to the final destination and place the object.

### Key Systems to Build

You will need to integrate several ROS 2 systems to achieve this:

-   **🎙️ Speech-to-Text System:** A node that uses Whisper to convert voice to text.
-   **🧠 Planning System:** A node that uses an LLM to convert text commands into an actionable plan.
-   **🗺️ Navigation System:** A properly configured Nav2 stack that can guide your robot through the environment.
-   **👀 Perception System:** A node that uses computer vision to detect and locate the objects required for the task. This could be a classic CV model or one trained on synthetic data.
-   **🦾 Manipulation System:** Action servers that control the robot's arm and gripper to perform `pick_up` and `place` motions.
-   **▶️ Executive Controller:** The main "Orchestrator" node that manages the state machine, calling the other systems in the correct order to execute the plan.

### Evaluation Criteria

Your project will be evaluated on:
-   **Success Rate:** Does the robot successfully complete the task from start to finish?
-   **Robustness:** Can the robot handle slight variations in object placement or starting position?
-   **Modularity:** Is your ROS 2 architecture well-designed, with clear separation between nodes and their responsibilities?
-   **Code Quality:** Is your code clean, well-commented, and efficient?

This project will provide a comprehensive portfolio piece, demonstrating your mastery of the entire robotics software stack, from low-level control to high-level AI-driven reasoning. Good luck!
