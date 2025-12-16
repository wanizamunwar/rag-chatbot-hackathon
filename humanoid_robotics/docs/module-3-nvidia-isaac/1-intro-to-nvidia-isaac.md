---
title: Introduction to NVIDIA Isaac
---

# 🚀 Introduction to NVIDIA Isaac

NVIDIA Isaac is an end-to-end platform designed to accelerate the entire lifecycle of robotics development, from training and simulation to deployment on real hardware. It's built on top of NVIDIA's core technologies, including CUDA, TensorRT, and the Omniverse platform, to provide GPU-accelerated performance at every stage.

The Isaac ecosystem is vast, but it can be broken down into a few key pillars:

### 1. Isaac Sim

Built on the **NVIDIA Omniverse™** platform, Isaac Sim is a photorealistic, physics-accurate robotics simulator. Its key differentiator is its focus on creating beautiful, realistic virtual worlds that are ideal for training and testing AI perception models. It features real-time ray tracing, accurate material physics, and a modular architecture.

### 2. Isaac ROS

Isaac ROS is a collection of hardware-accelerated ROS 2 packages (or "Gems") that are highly optimized for NVIDIA's Jetson and GPU hardware. These packages provide turnkey solutions for common robotics tasks, allowing developers to achieve high performance with minimal effort. This includes packages for:
-   Visual Odometry
-   Object Detection
-   Stereo Depth Estimation
-   AprilTag Detection

### 3. Isaac Gym

Isaac Gym is a specialized, high-performance reinforcement learning (RL) framework. It's designed for end-to-end GPU acceleration, allowing you to run thousands of parallel simulations directly on the GPU. This massively parallel approach dramatically speeds up the process of training complex RL policies for tasks like locomotion and manipulation.

### 4. NVIDIA TAO Toolkit

The TAO (Train, Adapt, and Optimize) Toolkit simplifies the process of creating production-ready AI models. It allows you to take pre-trained models from NVIDIA's NGC catalog and fine-tune them with your own custom data, all without needing deep AI expertise. The resulting models are optimized for inference on NVIDIA hardware.

Together, these tools form a cohesive platform for building the next generation of AI-powered robots. In this module, we will focus primarily on Isaac Sim and Isaac ROS.
