---
title: Isaac ROS Gems
---

# 💎 Isaac ROS Gems

**Isaac ROS Gems** are a collection of high-performance ROS 2 packages designed by NVIDIA to provide turnkey solutions for common robotics perception tasks. The key feature of these packages is that they are **hardware accelerated**, leveraging NVIDIA GPUs and Jetson platforms to achieve significantly higher throughput and lower latency than their CPU-based counterparts.

This allows a resource-constrained robot to perform complex perception tasks in real-time.

### The NITROS Pipeline

At the core of Isaac ROS is **NITROS** (NVIDIA Isaac Transport for ROS), a framework that optimizes the data pipeline. In a typical ROS application, image data is copied multiple times as it moves between the CPU and GPU. NITROS avoids this by keeping data on the GPU as much as possible, using technologies like **zero-copy** to pass data between different nodes without expensive memory transfers.

This results in a dramatic performance increase for perception pipelines.

![NITROS vs CPU Pipeline](https://developer-blogs.nvidia.com/wp-content/uploads/2022/03/isaac-ros-dp-nitros-based-pipeline.png)
*A NITROS pipeline (top) avoids costly CPU/GPU memory copies compared to a standard ROS pipeline (bottom).*

### Key Isaac ROS Packages

Isaac ROS provides a growing number of packages for common robotics tasks. Some of the most important include:

-   **`isaac_ros_visual_slam`**: This package provides a real-time Visual-Inertial Odometry (VIO) pipeline. It takes in stereo camera images and IMU data and produces a robust and accurate estimate of the robot's position and a map of the environment. This is a critical component for any autonomous robot.

-   **`isaac_ros_apriltag`**: AprilTags are fiducial markers (like QR codes) often used in robotics for localization and object tracking. This package provides a highly optimized node for detecting AprilTags in an image stream and determining their 3D pose relative to the camera.

-   **`isaac_ros_depth_image_proc`**: This package provides tools for processing depth images, such as converting a stereo image pair into a point cloud, which is a fundamental data structure for 3D perception.

-   **`isaac_ros_object_detection`**: Provides an easy way to deploy object detection models (like DetectNetv2 or YOLO) trained with the NVIDIA TAO Toolkit. It subscribes to an image topic and publishes the bounding boxes of detected objects.

By using these pre-built, accelerated packages, you can assemble a production-grade perception system with a fraction of the effort it would take to build one from scratch.
