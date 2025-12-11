---
sidebar_position: 2
description: "Learn about collision detection and configuration in Gazebo for humanoid robotics"
---

# Lesson 2: Collision Detection and Configuration

## Overview

In this lesson, we'll explore collision detection in Gazebo, which is crucial for realistic physics simulation. Properly configured collisions allow our humanoid robot to interact realistically with its environment.

## Understanding Collision Detection

Collision detection in Gazebo involves:

- **Collision Shapes**: Geometric representations used for collision detection
- **Collision Materials**: Physical properties like friction and restitution
- **Contact Detection**: How objects respond when they touch

### Collision vs Visual Geometry

It's important to distinguish between:
- **Visual Geometry**: What you see (for rendering)
- **Collision Geometry**: What you feel (for physics simulation)

Collision geometry is often simplified compared to visual geometry for performance.

## Adding Obstacles to the World

Let's enhance our world file by adding various obstacle shapes:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="collision_world">
    <!-- Include the default ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include the default sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics parameters -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Simple box obstacle -->
    <model name="box_obstacle">
      <pose>1 0 0.2 0 0 0</pose>
      <link name="box_link">
        <inertial>
          <mass>2.0</mass>
          <inertia>
            <ixx>0.067</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.067</iyy>
            <iyz>0.0</iyz>
            <izz>0.067</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.4 0.4 0.4</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.4 0.4 0.4</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Cylinder obstacle -->
    <model name="cylinder_obstacle">
      <pose>-1 0 0.5 0 0 0</pose>
      <link name="cylinder_link">
        <inertial>
          <mass>1.5</mass>
          <inertia>
            <ixx>0.077</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.077</iyy>
            <iyz>0.0</iyz>
            <izz>0.05</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Sphere obstacle -->
    <model name="sphere_obstacle">
      <pose>0 1 0.3 0 0 0</pose>
      <link name="sphere_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.02</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.02</iyy>
            <iyz>0.0</iyz>
            <izz>0.02</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.3</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.8 1</ambient>
            <diffuse>0.2 0.2 0.8 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.3</radius>
            </sphere>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## Configuring Collision Materials

Collision materials determine how objects interact physically. Here are the key parameters:

### Friction Properties

```xml
<surface>
  <friction>
    <ode>
      <mu>0.5</mu>      <!-- Primary friction coefficient -->
      <mu2>0.5</mu2>    <!-- Secondary friction coefficient -->
      <slip1>0.0</slip1> <!-- Primary slip coefficient -->
      <slip2>0.0</slip2> <!-- Secondary slip coefficient -->
    </ode>
  </friction>
</surface>
```

### Bounce Properties

```xml
<surface>
  <bounce>
    <restitution_coefficient>0.2</restitution_coefficient>
    <threshold>100000</threshold>
  </bounce>
</surface>
```

### Complete Model with Collision Configuration

Here's a more complex humanoid robot model with proper collision configuration:

```xml
<?xml version="1.0" ?>
<robot name="collision_humanoid">
  <!-- Base link with collision -->
  <link name="base_link">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.5"/>
      <inertia ixx="0.42" ixy="0.0" ixz="0.0" iyy="0.42" iyz="0.0" izz="0.8"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <cylinder length="1.0" radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <cylinder length="1.0" radius="0.1"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.8</mu>
            <mu2>0.8</mu2>
          </ode>
        </friction>
        <bounce>
          <restitution_coefficient>0.1</restitution_coefficient>
        </bounce>
      </surface>
    </collision>
  </link>

  <!-- Head link with collision -->
  <link name="head_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.00166" ixy="0.0" ixz="0.0" iyy="0.00166" iyz="0.0" izz="0.00166"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.5</mu>
            <mu2>0.5</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
  </link>

  <!-- Arm link with collision -->
  <link name="arm_link">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0.15 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0.15 0 0"/>
      <geometry>
        <capsule length="0.2" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0.15 0 0"/>
      <geometry>
        <capsule length="0.2" radius="0.05"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.6</mu>
            <mu2>0.6</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
  </link>

  <!-- Joint connecting base to head -->
  <joint name="base_to_head" type="fixed">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 1.0"/>
  </joint>

  <!-- Joint connecting base to arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0.1 0 0.5"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Collision Testing

To test collision detection:

1. Create a world file with the obstacles above
2. Add a moving robot model
3. Launch Gazebo: `gazebo path/to/your/collision_world.sdf`
4. Observe how the robot interacts with obstacles

## Collision Debugging Tips

1. **Enable Contact Visualization**: In Gazebo GUI, go to View → Contacts to see collision points
2. **Check Inertial Properties**: Ensure mass and inertia are properly defined
3. **Verify Geometry**: Make sure collision geometry matches visual geometry appropriately
4. **Adjust Time Step**: If collisions are unstable, try reducing `max_step_size`

## Diagram: Collision Detection Process

```
[Object A]     [Object B]
     ↓             ↓
[Collision] ←→ [Collision]
    ↓              ↓
[Contact Point] → [Response Force]
    ↓
[Physics Update]
```

## Test Steps

1. Create the world file with obstacles
2. Add the collision-configured robot model
3. Launch Gazebo and verify collision detection
4. Test robot movement around obstacles
5. Verify realistic collision responses

## Summary

In this lesson, you've learned about collision detection in Gazebo and how to properly configure collision shapes and materials for humanoid robots. Proper collision configuration is essential for realistic physics simulation.

## Next Steps

In the next lesson, we'll explore Unity rendering and how to create matching environments that visualize the physics simulation.