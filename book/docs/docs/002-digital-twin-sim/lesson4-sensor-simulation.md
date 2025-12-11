---
sidebar_position: 4
description: "Learn about sensor simulation in Gazebo: LiDAR, Depth Camera, and IMU for humanoid robotics"
---

# Lesson 4: Sensor Simulation in Gazebo

## Overview

In this lesson, we'll explore sensor simulation in Gazebo, which is essential for creating realistic humanoid robotics simulations. We'll cover LiDAR, depth camera, and IMU sensors that provide the robot with environmental awareness.

## Understanding Sensor Simulation

Sensor simulation in Gazebo involves:

- **Physics-based sensor models**: Realistic sensor behavior based on physics
- **Noise modeling**: Realistic sensor noise and inaccuracies
- **Data publishing**: Integration with ROS 2 for data access
- **Performance considerations**: Balancing realism with simulation speed

## LiDAR Sensor Simulation

LiDAR (Light Detection and Ranging) sensors provide 2D or 3D distance measurements.

### LiDAR Configuration in SDF

```xml
<!-- LiDAR sensor configuration -->
<sensor name="lidar_sensor" type="ray">
  <pose>0.2 0 0.5 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle> <!-- -π radians -->
        <max_angle>3.14159</max_angle>   <!-- π radians -->
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=lidar_scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_frame</frame_name>
    <update_rate>10</update_rate>
  </plugin>
</sensor>
```

### LiDAR ROS 2 Subscriber Example

```python
#!/usr/bin/env python3
"""
LiDAR sensor subscriber example
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/robot/lidar_scan',
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning

    def lidar_callback(self, msg):
        # Process LiDAR data
        ranges = np.array(msg.ranges)

        # Filter out invalid readings
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]

        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            print(f"Closest obstacle: {min_distance:.2f}m")

            # Example: detect obstacles within 1 meter
            obstacles = valid_ranges[valid_ranges < 1.0]
            if len(obstacles) > 0:
                print(f"Obstacles detected: {len(obstacles)} readings under 1m")

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()

    try:
        rclpy.spin(lidar_subscriber)
    except KeyboardInterrupt:
        print("Stopping LiDAR subscriber...")
    finally:
        lidar_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Depth Camera Simulation

Depth cameras provide 3D point cloud data and depth information.

### Depth Camera Configuration in SDF

```xml
<!-- Depth camera sensor configuration -->
<sensor name="depth_camera" type="depth">
  <pose>0.2 0 0.8 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees in radians -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <ros>
      <namespace>/robot</namespace>
    </ros>
    <camera_name>depth_camera</camera_name>
    <frame_name>depth_camera_frame</frame_name>
    <min_depth>0.1</min_depth>
    <max_depth>10.0</max_depth>
    <update_rate>15</update_rate>
    <image_topic_name>/image_raw</image_topic_name>
    <depth_image_topic_name>/depth/image_raw</depth_image_topic_name>
    <point_cloud_topic_name>/points</point_cloud_topic_name>
  </plugin>
</sensor>
```

### Depth Camera ROS 2 Subscriber Example

```python
#!/usr/bin/env python3
"""
Depth camera subscriber example
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthCameraSubscriber(Node):
    def __init__(self):
        super().__init__('depth_camera_subscriber')
        self.bridge = CvBridge()

        # Subscribe to depth image
        self.depth_subscription = self.create_subscription(
            Image,
            '/robot/depth_camera/depth/image_raw',
            self.depth_image_callback,
            10)
        self.depth_subscription  # prevent unused variable warning

        # Subscribe to RGB image
        self.rgb_subscription = self.create_subscription(
            Image,
            '/robot/depth_camera/image_raw',
            self.rgb_image_callback,
            10)
        self.rgb_subscription  # prevent unused variable warning

    def depth_image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

            # Get depth at center of image
            height, width = cv_image.shape
            center_depth = cv_image[height//2, width//2]

            if not np.isnan(center_depth) and center_depth > 0:
                print(f"Distance at center: {center_depth:.2f}m")

            # Find minimum distance in image (closest object)
            valid_depths = cv_image[np.isfinite(cv_image) & (cv_image > 0)]
            if len(valid_depths) > 0:
                min_depth = np.min(valid_depths)
                print(f"Closest object: {min_depth:.2f}m")

        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def rgb_image_callback(self, msg):
        # Convert and display RGB image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # In a real application, you might process this image
            # For now, just acknowledge receipt
            self.get_logger().info('Received RGB image')
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {e}')

def main(args=None):
    rclpy.init(args=args)
    depth_camera_subscriber = DepthCameraSubscriber()

    try:
        rclpy.spin(depth_camera_subscriber)
    except KeyboardInterrupt:
        print("Stopping depth camera subscriber...")
    finally:
        depth_camera_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## IMU Sensor Simulation

IMU (Inertial Measurement Unit) sensors provide orientation, angular velocity, and linear acceleration.

### IMU Configuration in SDF

```xml
<!-- IMU sensor configuration -->
<sensor name="imu_sensor" type="imu">
  <pose>0 0 0.5 0 0 0</pose>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/robot</namespace>
    </ros>
    <frame_name>imu_frame</frame_name>
    <update_rate>100</update_rate>
  </plugin>
</sensor>
```

### IMU ROS 2 Subscriber Example

```python
#!/usr/bin/env python3
"""
IMU sensor subscriber example
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/robot/imu/data',
            self.imu_callback,
            10)
        self.subscription  # prevent unused variable warning

    def imu_callback(self, msg):
        # Extract orientation (using quaternion to Euler conversion)
        orientation = msg.orientation
        euler = self.quaternion_to_euler(orientation)

        # Extract angular velocity
        angular_velocity = msg.angular_velocity

        # Extract linear acceleration
        linear_acceleration = msg.linear_acceleration

        print(f"Orientation: Roll={euler.x:.2f}, Pitch={euler.y:.2f}, Yaw={euler.z:.2f}")
        print(f"Angular Vel: X={angular_velocity.x:.2f}, Y={angular_velocity.y:.2f}, Z={angular_velocity.z:.2f}")
        print(f"Linear Acc: X={linear_acceleration.x:.2f}, Y={linear_acceleration.y:.2f}, Z={linear_acceleration.z:.2f}")

    def quaternion_to_euler(self, quaternion):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return Vector3(x=roll, y=pitch, z=yaw)

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()

    try:
        rclpy.spin(imu_subscriber)
    except KeyboardInterrupt:
        print("Stopping IMU subscriber...")
    finally:
        imu_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete Robot Model with All Sensors

Here's a complete URDF model that includes all three sensor types:

```xml
<?xml version="1.0" ?>
<robot name="sensor_robot">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <cylinder length="1.0" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <cylinder length="1.0" radius="0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- LiDAR mount link -->
  <link name="lidar_mount">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- LiDAR sensor link -->
  <link name="lidar_frame">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Depth camera mount link -->
  <link name="camera_mount">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </visual>
  </link>

  <!-- Depth camera link -->
  <link name="depth_camera_frame">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- IMU link -->
  <link name="imu_frame">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="base_to_lidar_mount" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_mount"/>
    <origin xyz="0.2 0 0.8"/>
  </joint>

  <joint name="lidar_mount_to_frame" type="fixed">
    <parent link="lidar_mount"/>
    <child link="lidar_frame"/>
    <origin xyz="0 0 0.025"/>
  </joint>

  <joint name="base_to_camera_mount" type="fixed">
    <parent link="base_link"/>
    <child link="camera_mount"/>
    <origin xyz="0.2 0 0.7"/>
  </joint>

  <joint name="camera_mount_to_frame" type="fixed">
    <parent link="camera_mount"/>
    <child link="depth_camera_frame"/>
    <origin xyz="0 0 0.025"/>
  </joint>

  <joint name="base_to_imu" type="fixed">
    <parent link="base_link"/>
    <child link="imu_frame"/>
    <origin xyz="0 0 0.5"/>
  </joint>

  <!-- Gazebo plugins for sensors -->
  <gazebo reference="lidar_frame">
    <sensor name="lidar_sensor" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/robot</namespace>
          <remapping>~/out:=lidar_scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_frame</frame_name>
        <update_rate>10</update_rate>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="depth_camera_frame">
    <sensor name="depth_camera" type="depth">
      <pose>0 0 0 0 0 0</pose>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
        <ros>
          <namespace>/robot</namespace>
        </ros>
        <camera_name>depth_camera</camera_name>
        <frame_name>depth_camera_frame</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>10.0</max_depth>
        <update_rate>15</update_rate>
        <image_topic_name>/image_raw</image_topic_name>
        <depth_image_topic_name>/depth/image_raw</depth_image_topic_name>
        <point_cloud_topic_name>/points</point_cloud_topic_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_frame">
    <sensor name="imu_sensor" type="imu">
      <pose>0 0 0 0 0 0</pose>
      <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <namespace>/robot</namespace>
        </ros>
        <frame_name>imu_frame</frame_name>
        <update_rate>100</update_rate>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

## Sensor Data Logging

To log sensor data for analysis:

```python
#!/usr/bin/env python3
"""
Sensor data logger for analysis
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import numpy as np
import csv
import os
from datetime import datetime

class SensorLogger(Node):
    def __init__(self):
        super().__init__('sensor_logger')

        # Create directory for logs
        self.log_dir = f"sensor_logs_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.makedirs(self.log_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.lidar_data_count = 0
        self.imu_data_count = 0
        self.image_data_count = 0

        # Subscribe to all sensor topics
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/robot/lidar_scan', self.lidar_callback, 10)
        self.imu_subscription = self.create_subscription(
            Imu, '/robot/imu/data', self.imu_callback, 10)
        self.image_subscription = self.create_subscription(
            Image, '/robot/depth_camera/image_raw', self.image_callback, 10)

    def lidar_callback(self, msg):
        # Log LiDAR data
        timestamp = self.get_clock().now().seconds_nanoseconds()
        filename = os.path.join(self.log_dir, f"lidar_{self.lidar_data_count:05d}.csv")

        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp_sec', 'timestamp_nanosec', 'range_min', 'range_max', 'ranges'])
            writer.writerow([timestamp[0], timestamp[1], msg.range_min, msg.range_max,
                           ','.join([str(r) for r in msg.ranges])])

        self.lidar_data_count += 1
        if self.lidar_data_count % 100 == 0:
            self.get_logger().info(f'Logged {self.lidar_data_count} LiDAR samples')

    def imu_callback(self, msg):
        # Log IMU data
        timestamp = self.get_clock().now().seconds_nanoseconds()
        filename = os.path.join(self.log_dir, f"imu_{self.imu_data_count:05d}.csv")

        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp_sec', 'timestamp_nanosec', 'orientation_x', 'orientation_y',
                           'orientation_z', 'orientation_w', 'angular_velocity_x', 'angular_velocity_y',
                           'angular_velocity_z', 'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'])
            writer.writerow([timestamp[0], timestamp[1],
                           msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                           msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                           msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        self.imu_data_count += 1
        if self.imu_data_count % 100 == 0:
            self.get_logger().info(f'Logged {self.imu_data_count} IMU samples')

    def image_callback(self, msg):
        # Save image (every 10th image to reduce storage)
        if self.image_data_count % 10 == 0:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                filename = os.path.join(self.log_dir, f"image_{self.image_data_count:05d}.jpg")
                cv2.imwrite(filename, cv_image)
            except Exception as e:
                self.get_logger().error(f'Error saving image: {e}')

        self.image_data_count += 1
        if self.image_data_count % 100 == 0:
            self.get_logger().info(f'Processed {self.image_data_count} images')

def main(args=None):
    rclpy.init(args=args)
    sensor_logger = SensorLogger()

    try:
        rclpy.spin(sensor_logger)
    except KeyboardInterrupt:
        print("Stopping sensor logger...")
    finally:
        sensor_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Performance Considerations

| Sensor Type | Update Rate | Performance Impact | Typical Use |
|-------------|-------------|-------------------|-------------|
| LiDAR | 5-20 Hz | Medium | Obstacle detection, mapping |
| Depth Camera | 15-30 Hz | High | 3D reconstruction, navigation |
| IMU | 100-200 Hz | Low | Orientation, motion detection |

## Diagram: Sensor Data Flow

```
[Physical World] → [Gazebo Sensors] → [ROS 2 Topics] → [Subscriber Nodes]
       ↓                ↓                   ↓                ↓
[Physics Engine] → [Sensor Plugins] → [sensor_msgs] → [Processing Logic]
```

## Test Steps

1. Add sensor configurations to your robot model
2. Launch Gazebo with the sensor-equipped robot
3. Run the ROS 2 subscriber examples to verify sensor data
4. Check sensor data quality and update rates
5. Test sensor integration with navigation algorithms

## Summary

In this lesson, you've learned how to simulate LiDAR, depth camera, and IMU sensors in Gazebo, including configuration, ROS 2 integration, and data processing examples. These sensors provide essential environmental awareness for humanoid robotics applications.

## Next Steps

With all four lessons completed, you now have a complete digital twin system with physics simulation, collision detection, visualization, and sensor simulation. The next step would be to integrate all components and create the bridge between Gazebo and Unity for a complete digital twin experience.