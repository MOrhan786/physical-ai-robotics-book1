# Quickstart Guide: Digital Twin Simulation (Gazebo & Unity)

**Feature**: 002-digital-twin-sim
**Date**: 2025-12-10
**Status**: Draft

## Overview

This quickstart guide provides the essential steps to set up and run the Digital Twin Simulation environment with Gazebo physics and Unity rendering. This setup allows you to experiment with humanoid robot simulation, sensor simulation, and human-robot interaction.

## Prerequisites

### System Requirements
- **OS**: Ubuntu 20.04/22.04, Windows 10/11 (with WSL2), or macOS 12+
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 10GB free space
- **GPU**: Dedicated GPU recommended for Unity rendering (Integrated graphics may work but with reduced performance)

### Software Dependencies
1. **ROS 2**: Humble Hawksbill or Iron Irwini
2. **Gazebo**: Gazebo Classic 11.x
3. **Unity**: Unity Hub with Unity 2022.3 LTS
4. **Python**: 3.8-3.10
5. **Git**: Version control system

## Installation Steps

### 1. Install ROS 2
```bash
# For Ubuntu
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### 2. Install Gazebo Classic
```bash
sudo apt install gazebo libgazebo-dev
# Verify installation
gazebo --version
```

### 3. Install Unity
1. Download and install Unity Hub from https://unity.com/
2. Through Unity Hub, install Unity 2022.3 LTS
3. Install additional modules if prompted for Linux/Windows build support

### 4. Clone the Repository
```bash
git clone https://github.com/your-repo/physical-ai-robotics-book.git
cd physical-ai-robotics-book
```

### 5. Set up ROS 2 Workspace
```bash
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws/src
ln -s /path/to/physical-ai-robotics-book/book/examples/002-digital-twin-sim ~/digital_twin_ws/src/digital_twin_examples
cd ~/digital_twin_ws
colcon build
source install/setup.bash
```

### 6. Install Unity ROS TCP Connector
1. Open Unity Hub and create a new 3D project
2. In Unity, go to Window → Package Manager
3. Click the + button → Add package from git URL
4. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git`

## Running the Basic Simulation

### 1. Start Gazebo Simulation
```bash
cd ~/digital_twin_ws
source install/setup.bash
ros2 launch digital_twin_examples simulation.launch.py
```

### 2. Start ROS Bridge
```bash
# In a new terminal
cd ~/digital_twin_ws
source install/setup.bash
ros2 run digital_twin_examples ros2_bridge.py
```

### 3. Start Unity Scene
1. Open Unity Hub
2. Open the project in `book/examples/002-digital-twin-sim/unity_scenes`
3. Load the "basic_robot_scene.unity" scene
4. Run the scene (Play button)

## Basic Examples

### Example 1: Run Basic Gazebo Simulation
```bash
# Launch the basic world with a simple robot
ros2 launch digital_twin_examples simulation.launch.py world:=basic_environment
```

### Example 2: Test LiDAR Sensor
```bash
# Launch with LiDAR sensor enabled
ros2 launch digital_twin_examples sensors.launch.py sensor_type:=lidar
# Subscribe to LiDAR data
ros2 topic echo /robot/lidar_scan sensor_msgs/msg/LaserScan
```

### Example 3: Unity-ROS Connection
1. Make sure ROS bridge is running
2. In Unity scene, check that "ROS Connection" object is properly configured
3. Verify connection status in Unity console

## Troubleshooting

### Common Issues

**Q: Gazebo fails to start with graphics errors**
A: Try running with software rendering:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
gazebo
```

**Q: Unity scene appears empty or missing models**
A: Check that assets are properly imported and materials are assigned.

**Q: ROS bridge fails to connect**
A: Verify IP addresses and ports match between ROS bridge and Unity connector.

**Q: Simulation runs slowly**
A: Reduce physics update rate or simplify scene complexity.

## Next Steps

1. Follow the detailed chapters to learn about physics simulation
2. Explore sensor configuration and data processing
3. Build your own Unity environments
4. Implement human-robot interaction scenarios
5. Connect to real ROS 2 systems

## Useful Commands

```bash
# Check available ROS topics
ros2 topic list

# Check robot state
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Visualize TF frames
rviz2

# Monitor simulation performance
gz stats
```