# Research: Digital Twin Simulation (Gazebo & Unity)

**Feature**: 002-digital-twin-sim
**Date**: 2025-12-10
**Status**: In Progress

## Research Goals

This research document captures the investigation and analysis needed to implement the Digital Twin Simulation module. The focus is on understanding Gazebo physics simulation, Unity rendering, sensor simulation, and the integration between these systems via ROS 2.

## Key Research Areas

### 1. Gazebo Physics Simulation

**Objective**: Determine optimal configuration for humanoid robot physics simulation

**Findings**:
- Gazebo Classic (Gazebo 11) vs Ignition Fortress: Gazebo Classic has more extensive documentation and ROS 2 bridge support, but Ignition offers better performance and modern features
- Physics engine options: ODE (default), Bullet, DART - ODE is most stable for humanoid robots
- Gravity setting: Standard 9.81 m/s² for Earth-like simulation
- Realistic humanoid parameters:
  - Mass: 30-80 kg depending on robot size
  - Inertia: Calculated based on geometric approximations
  - Friction: 0.5-0.9 for typical surfaces
  - Damping: Low values (0.01-0.1) for realistic movement

**References**:
- Gazebo Documentation: http://gazebosim.org/
- ROS 2 with Gazebo: https://github.com/ros-simulation/gazebo_ros_pkgs

### 2. Sensor Simulation

**Objective**: Understand realistic simulation of LiDAR, depth camera, and IMU sensors

**LiDAR Simulation**:
- Plugin: libgazebo_ros_lidar.so
- Typical parameters:
  - Range: 0.12-30m
  - Resolution: 0.25-1.0 degrees
  - Scan rate: 5-20Hz
  - Noise: Gaussian with std_dev 0.01-0.05m

**Depth Camera Simulation**:
- Plugin: libgazebo_ros_openni_kinect.so
- Typical parameters:
  - Resolution: 640x480 or 1280x720
  - FOV: 57-60 degrees horizontal
  - Range: 0.1-10m
  - Noise: Gaussian for depth accuracy

**IMU Simulation**:
- Plugin: libgazebo_ros_imu_sensor.so
- Typical parameters:
  - Update rate: 100-200Hz
  - Noise: Linear acceleration and angular velocity noise models
  - Bias: Initial bias and drift parameters

### 3. Unity Integration

**Objective**: Determine best approach for Unity-ROS 2 integration

**Options**:
1. Unity ROS TCP Connector (Recommended)
   - Pro: Well-documented, maintained by Unity Robotics
   - Con: Network-based communication, potential latency
   - URL: https://github.com/Unity-Technologies/ROS-TCP-Connector

2. Custom ROS 2 Unity Integration
   - Pro: Direct integration, potentially lower latency
   - Con: Complex to implement, limited documentation
   - Requires: Custom message serialization

**Coordinate System Alignment**:
- Gazebo: Right-handed coordinate system (X forward, Y left, Z up)
- Unity: Left-handed coordinate system (X right, Y up, Z forward)
- Transformation needed: Rotate and convert coordinate systems

### 4. ROS 2 Bridge Architecture

**Objective**: Design efficient communication between simulation and ROS 2

**Standard Messages**:
- Sensor data: sensor_msgs/LaserScan, sensor_msgs/Image, sensor_msgs/Imu
- Robot control: geometry_msgs/Twist, sensor_msgs/JointState
- TF transforms: tf2_msgs/TFMessage

**Bridge Configuration**:
- Bidirectional communication
- Topic mapping between Gazebo and ROS 2
- Synchronization mechanisms

### 5. Performance Optimization

**Key Considerations**:
- Physics update rate: 1000Hz for stable simulation
- Rendering rate: 30-60 FPS for acceptable visual quality
- Sensor update rates: Match real-world sensor capabilities
- Memory usage: Monitor for extended simulation sessions
- Hardware requirements: Target educational environments (mid-range laptops/desktops)

## Architecture Decisions Summary

1. **Gazebo Version**: Gazebo Classic (11.x) for stability and documentation
2. **Physics Engine**: ODE with standard humanoid parameters
3. **Sensor Plugins**: Standard Gazebo ROS plugins for LiDAR, depth camera, IMU
4. **Unity Integration**: Unity ROS TCP Connector for proven reliability
5. **Coordinate System**: Implement transformation layer to align Gazebo and Unity coordinates

## Next Steps

1. Create proof-of-concept for each component
2. Test integration between Gazebo and Unity
3. Validate sensor simulation accuracy
4. Optimize performance for target hardware
5. Document configuration parameters and best practices