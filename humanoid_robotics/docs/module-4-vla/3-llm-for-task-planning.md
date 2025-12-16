---
title: LLMs for Task Planning
---

# 🤖 LLMs for Task Planning

Once we have a transcribed text command like, "bring me the soda from the kitchen," the robot faces a new challenge: what does that *mean* in terms of actions? This is where **cognitive planning** using Large Language Models (LLMs) comes in. An LLM can act as the robot's "brain," breaking down a high-level goal into a sequence of concrete, executable steps.

### Prompt Engineering for Robotics

The key to using an LLM for task planning is **prompt engineering**. We can't just send the raw text command to the LLM. Instead, we must provide it with context in the prompt, telling it what "tools" it has available.

Our prompt to the LLM will typically include:
1.  **The Goal:** The natural language command from the user.
2.  **The Environment:** A simplified text description of the current scene (e.g., "Objects in view: table, chair, red_block."). This can come from a perception system.
3.  **Available Functions:** A list of the high-level actions the robot can perform, described like functions in a programming language.

**Example Prompt:**
```text
You are a helpful robot assistant. Your goal is to achieve the user's command by outputting a sequence of functions to call.

## Environment
- You see: table, chair, red_block, blue_bowl.
- The gripper is empty.

## Available Functions
- navigate_to(location): Moves the robot to a location.
- pick_up(object_id): Picks up an object.
- place(object_id, location_id): Places an object in/on another object.

## User Command
"pick up the red block and put it in the blue bowl"

## Plan
```

### Generating the Plan

Given the prompt above, a powerful LLM (like GPT-4) can reason about the request and generate a structured plan, often in a format like JSON or a simple list of function calls:

**LLM Output:**
```json
[
  {
    "function": "pick_up",
    "parameters": {
      "object_id": "red_block"
    }
  },
  {
    "function": "place",
    "parameters": {
      "object_id": "red_block",
      "location_id": "blue_bowl"
    }
  }
]
```

### The Execution Loop

A ROS 2 "Orchestrator" node would be responsible for this entire process:
1.  **Subscribe** to the `/voice_command` topic.
2.  Upon receiving a command, **construct the prompt** with the latest environment data.
3.  **Send the prompt** to the LLM API.
4.  **Parse the response** (e.g., the JSON plan).
5.  **Execute the plan** step-by-step by calling the appropriate ROS 2 Action servers (e.g., call the `navigate_to` action server, then the `pick_up` action server).

This architecture allows the robot to be incredibly flexible. To teach it a new skill, you simply need to create a new ROS action server and add its description to the "Available Functions" list in the LLM prompt. The LLM will automatically learn how to incorporate this new skill into its plans.
