# ROS Bridge Contract: Digital Twin Simulation

**Feature**: 002-digital-twin-sim
**Date**: 2025-12-10
**Status**: Draft

## Overview

This contract defines the interface and behavior of the ROS 2 bridge connecting Gazebo physics simulation to Unity rendering. The bridge facilitates bidirectional communication between the simulation environment and external ROS 2 nodes.

## Architecture

```
[ROS 2 Nodes] ↔ [ROS Bridge] ↔ [Gazebo Simulation] ↔ [Unity Renderer]
```

## API Endpoints

### 1. Robot Control Interface

**Topic**: `/robot/cmd_vel`
- **Type**: `geometry_msgs/Twist`
- **Direction**: Publisher (from ROS) / Subscriber (to Gazebo)
- **Purpose**: Send velocity commands to the robot base
- **Rate**: 50Hz
- **QoS**: Reliability: Reliable, Durability: Volatile

**Service**: `/robot/set_joint_positions`
- **Type**: Custom service (SetJointPositions)
- **Direction**: Request/Response
- **Purpose**: Set specific joint positions for the robot
- **Timeout**: 5 seconds

### 2. Sensor Data Interface

**Topic**: `/robot/lidar_scan`
- **Type**: `sensor_msgs/LaserScan`
- **Direction**: Publisher (from Gazebo) / Subscriber (to ROS)
- **Purpose**: Publish LiDAR sensor data
- **Rate**: 10Hz
- **QoS**: Reliability: Best Effort, Durability: Volatile

**Topic**: `/robot/depth_camera/image_raw`
- **Type**: `sensor_msgs/Image`
- **Direction**: Publisher (from Gazebo) / Subscriber (to ROS)
- **Purpose**: Publish depth camera image data
- **Rate**: 15Hz
- **QoS**: Reliability: Best Effort, Durability: Volatile

**Topic**: `/robot/imu/data`
- **Type**: `sensor_msgs/Imu`
- **Direction**: Publisher (from Gazebo) / Subscriber (to ROS)
- **Purpose**: Publish IMU sensor data
- **Rate**: 100Hz
- **QoS**: Reliability: Best Effort, Durability: Volatile

### 3. State Information Interface

**Topic**: `/robot/joint_states`
- **Type**: `sensor_msgs/JointState`
- **Direction**: Publisher (from Gazebo) / Subscriber (to ROS)
- **Purpose**: Publish current joint positions, velocities, and efforts
- **Rate**: 50Hz
- **QoS**: Reliability: Best Effort, Durability: Volatile

**Topic**: `/tf` and `/tf_static`
- **Type**: `tf2_msgs/TFMessage`
- **Direction**: Publisher (from Gazebo) / Subscriber (to ROS)
- **Purpose**: Publish transform relationships between frames
- **Rate**: 50Hz
- **QoS**: Reliability: Reliable, Durability: Volatile

## Unity Interface

### TCP Connection Parameters

**Default Host**: `127.0.0.1`
**Default Port**: `10000`
**Protocol**: TCP/IP
**Message Format**: JSON over TCP

### Unity Message Types

**RobotPoseMessage**:
```json
{
  "type": "RobotPose",
  "robot_name": "string",
  "position": {"x": float, "y": float, "z": float},
  "orientation": {"x": float, "y": float, "z": float, "w": float},
  "timestamp": float
}
```

**SensorDataMessage**:
```json
{
  "type": "SensorData",
  "sensor_name": "string",
  "sensor_type": "lidar|camera|imu",
  "data": object,
  "timestamp": float
}
```

## Performance Guarantees

### Latency Requirements
- ROS Bridge to Gazebo: <10ms average
- Gazebo to Unity: <50ms average (due to rendering pipeline)
- End-to-end (ROS to Unity visualization): <100ms

### Throughput Requirements
- Maximum message rate: 1000 messages/second per topic
- Maximum data size: 1MB per message
- Connection stability: 99.9% uptime during simulation

### Error Handling
- Network disconnection recovery: Automatic reconnection with exponential backoff
- Message validation: Invalid messages are logged and dropped
- Timeout handling: 5-second timeout for service calls

## Security Considerations

### Authentication
- No authentication required for local development
- For network deployments, use ROS 2 security features (DDS Security)

### Data Validation
- All incoming messages are validated for proper format
- Malformed messages are rejected and logged
- Buffer overflow protection implemented

## Versioning

- **Contract Version**: 1.0.0
- **ROS 2 Compatibility**: Humble Hawksbill and Iron Irwini
- **Gazebo Compatibility**: Gazebo Classic 11.x
- **Unity Compatibility**: 2022.3 LTS and newer

## Testing Contract Compliance

### Required Tests
1. **Connection Test**: Verify TCP connection establishment and teardown
2. **Message Round-trip**: Verify message transmission in both directions
3. **Performance Test**: Verify latency and throughput requirements
4. **Error Recovery**: Verify behavior under network failure conditions
5. **Data Integrity**: Verify message format and content accuracy

### Test Scenarios
```bash
# Connection test
ros2 run digital_twin_examples test_bridge_connection

# Message round-trip test
ros2 run digital_twin_examples test_message_roundtrip

# Performance test
ros2 run digital_twin_examples test_performance --duration 60
```

## Implementation Notes

### Gazebo Plugin Implementation
- Use `gazebo_ros` plugin interface
- Implement proper lifecycle management
- Handle simulation reset events

### Unity Integration
- Use Unity ROS TCP Connector package
- Implement message serialization/deserialization
- Handle Unity scene lifecycle events

## Change Management

Any changes to this contract must:
1. Maintain backward compatibility where possible
2. Update all dependent components
3. Pass all contract compliance tests
4. Be documented with version updates