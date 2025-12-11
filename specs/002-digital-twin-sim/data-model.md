# Data Model: Digital Twin Simulation (Gazebo & Unity)

**Feature**: 002-digital-twin-sim
**Date**: 2025-12-10
**Status**: Draft

## Overview

This document defines the key data structures and entities used in the Digital Twin Simulation module. These entities represent the core concepts that will be implemented in both Gazebo physics simulation and Unity rendering environments.

## Core Entities

### 1. Simulation World

**Description**: Represents the complete simulation environment including physics properties, objects, and configuration.

**Attributes**:
- `name`: String identifier for the world
- `gravity`: 3D vector representing gravitational acceleration (typically [0, 0, -9.81])
- `physics_engine`: Name of the physics engine (ODE, Bullet, DART)
- `time_step`: Physics simulation time step (seconds)
- `real_time_factor`: Target simulation speed relative to real time
- `models`: Collection of models present in the world
- `lighting`: Environmental lighting configuration
- `materials`: Material properties for surfaces

**Relationships**:
- Contains many Models
- Uses Physics Engine configuration

### 2. Robot Model

**Description**: Represents a robot entity with physical properties, joints, links, and sensors.

**Attributes**:
- `name`: Unique identifier for the robot
- `urdf_path`: Path to URDF file defining the robot structure
- `initial_pose`: Position and orientation in the simulation world
- `mass`: Total mass of the robot (kg)
- `links`: Collection of rigid body parts
- `joints`: Collection of connections between links
- `sensors`: Collection of sensor attachments
- `controllers`: Collection of control interfaces

**Relationships**:
- Contains many Links
- Contains many Joints
- Contains many Sensors
- Connected to Controllers

### 3. Link

**Description**: A rigid body part of a robot model with physical properties.

**Attributes**:
- `name`: Unique identifier for the link
- `visual_mesh`: 3D mesh for rendering
- `collision_mesh`: Mesh for collision detection
- `inertial_properties`: Mass, center of mass, and inertia tensor
- `material`: Visual material properties
- `parent_joint`: Joint connecting to parent link (null for base link)

**Relationships**:
- Connected to one parent Joint
- Connected to many child Joints

### 4. Joint

**Description**: Connection between two links that defines their relative motion.

**Attributes**:
- `name`: Unique identifier for the joint
- `type`: Joint type (revolute, prismatic, fixed, continuous, etc.)
- `parent_link`: Link that is the parent in the kinematic chain
- `child_link`: Link that is the child in the kinematic chain
- `axis`: Axis of motion for the joint
- `limits`: Position, velocity, and effort limits
- `dynamics`: Damping and friction parameters

**Relationships**:
- Connected to one Parent Link
- Connected to one Child Link

### 5. Sensor

**Description**: Virtual sensor attached to a robot link that generates simulated data.

**Attributes**:
- `name`: Unique identifier for the sensor
- `type`: Sensor type (LiDAR, depth_camera, IMU, etc.)
- `parent_link`: Link to which the sensor is attached
- `pose`: Position and orientation relative to parent link
- `update_rate`: Rate at which sensor data is generated (Hz)
- `parameters`: Type-specific configuration parameters
- `topic`: ROS topic for sensor data output

**Subtypes**:
- **LiDAR Sensor**: Range, resolution, field of view parameters
- **Depth Camera**: Resolution, field of view, depth range
- **IMU Sensor**: Noise characteristics, update rate

**Relationships**:
- Attached to one Parent Link
- Outputs to ROS Topic

### 6. Simulation State

**Description**: Represents the current state of the simulation including all dynamic elements.

**Attributes**:
- `timestamp`: Current simulation time
- `robot_states`: Collection of robot poses and joint states
- `sensor_data`: Collection of latest sensor readings
- `world_properties`: Current world physics properties
- `environment_state`: State of dynamic environment elements

**Relationships**:
- Contains many Robot States
- Contains many Sensor Data readings

### 7. Unity Scene Configuration

**Description**: Configuration for Unity rendering that corresponds to the Gazebo simulation.

**Attributes**:
- `scene_name`: Name of the Unity scene
- `synchronization_mode`: How Unity visuals sync with Gazebo physics
- `rendering_quality`: Quality settings for graphics
- `camera_configurations`: Collection of camera setups
- `lighting_config`: Lighting parameters that match Gazebo
- `asset_mappings`: Mapping between Gazebo and Unity assets

**Relationships**:
- Corresponds to one Simulation World
- Contains many Camera Configurations

### 8. ROS Bridge Configuration

**Description**: Configuration for communication between Gazebo simulation and ROS 2.

**Attributes**:
- `connection_type`: Communication method (TCP, UDP, etc.)
- `topic_mappings`: Mapping between Gazebo and ROS topics
- `message_types`: ROS message types used for communication
- `update_rates`: Communication frequency settings
- `synchronization_settings`: Time synchronization parameters

**Relationships**:
- Connects Gazebo Simulation to ROS 2
- Maps to many Topic Mappings

## Data Flow Patterns

### Physics State Flow
```
Simulation World → Robot Model → Joint States → Link Positions → Physics Updates
```

### Sensor Data Flow
```
Simulation World → Sensor Simulation → Sensor Data → ROS Bridge → ROS Topics
```

### Rendering Synchronization
```
Physics State → State Publisher → Unity Connector → Unity Scene Updates
```

## Validation Rules

1. **Mass Conservation**: Total robot mass must equal sum of link masses
2. **Kinematic Consistency**: Joint constraints must be satisfied at all times
3. **Sensor Limits**: Sensor parameters must be within physically realistic ranges
4. **Coordinate Alignment**: Gazebo and Unity coordinates must be properly transformed
5. **Update Rate Consistency**: Sensor update rates must be achievable within performance constraints

## Assumptions

1. **Coordinate Systems**: Gazebo uses right-handed coordinates, Unity uses left-handed coordinates
2. **Time Synchronization**: Simulation time and rendering time are synchronized within acceptable tolerance
3. **Network Latency**: Unity-ROS communication has acceptable latency for real-time interaction
4. **Hardware Performance**: Target hardware can run simulation at required frame rates
5. **Asset Compatibility**: 3D models can be appropriately converted between Gazebo and Unity formats