---
title: Simulating Sensors
---

# 📡 Simulating Sensors

A robot's software is useless without data from sensors. A key feature of any good simulator is its ability to generate realistic sensor data. Gazebo provides a rich set of plugins for simulating a wide variety of common robotic sensors.

These simulated sensors are attached to a robot's URDF model and publish their data to ROS topics, just like real hardware would. This means your perception stack doesn't need to know whether it's running in simulation or on a real robot.

### 1. Cameras

Camera simulation is essential for testing any computer vision algorithm. The Gazebo camera plugin can simulate:
-   Monocular and stereo cameras.
-   Color, grayscale, and depth images.
-   Camera properties like resolution, frame rate, and field of view (FOV).
-   Lens distortion and sensor noise to better mimic real-world imperfections.

```xml
<!-- Example of a camera sensor in a URDF/SDF file -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.39626</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/demo</namespace>
        <image_topic>camera1/image_raw</image_topic>
        <camera_info_topic>camera1/camera_info</camera_info_topic>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```
This XML snippet defines a camera sensor that publishes ROS messages on the `/demo/camera1/image_raw` topic.

### 2. LiDAR and Laser Scanners

LiDAR (Light Detection and Ranging) is the workhorse sensor for mapping and navigation. Gazebo can simulate both 2D laser scanners and 3D LiDAR sensors. The plugins allow you to configure:
-   **Range:** The maximum and minimum distance the sensor can see.
-   **Resolution:** The number of points or rays in each scan.
-   **Update Rate:** How many scans are published per second.
-   **Noise:** Adding Gaussian noise to the range measurements to simulate real-world inaccuracy.

### 3. IMUs (Inertial Measurement Units)

An IMU is crucial for a robot to understand its orientation and motion. It typically combines an accelerometer, a gyroscope, and sometimes a magnetometer. The Gazebo IMU plugin simulates these effects, publishing `sensor_msgs/Imu` messages that contain:
-   Angular velocity.
-   Linear acceleration.
-   Orientation (as a quaternion).

By combining these simulated sensors, you can create a digital twin that provides a rich, multi-modal stream of data, allowing you to thoroughly test your robot's entire perception and autonomy stack before ever touching hardware.
