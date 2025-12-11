---
sidebar_position: 1
description: "Learn about Gazebo physics simulation for humanoid robotics"
---

# Lesson 1: Gazebo Physics Simulation

## Overview

In this lesson, we'll explore the fundamentals of physics simulation in Gazebo, which forms the core of our digital twin system. Gazebo provides realistic physics simulation including gravity, collisions, and joint dynamics that are essential for humanoid robotics.

## Understanding Gazebo Physics

Gazebo uses a physics engine (typically ODE - Open Dynamics Engine) to simulate realistic physical interactions. For humanoid robotics, this includes:

- **Gravity**: Downward acceleration at 9.81 m/s²
- **Collisions**: Detection and response between objects
- **Joint Dynamics**: Movement constraints and forces
- **Friction**: Surface interaction properties

### Key Physics Concepts

1. **Time Stepping**: Gazebo simulates physics at discrete time intervals
2. **Real-time Factor**: Controls simulation speed relative to real time
3. **Update Rates**: How frequently physics calculations occur

## Creating a Basic World

Let's create a simple world file that will serve as our physics environment:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="basic_physics_world">
    <!-- Include the default ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include the default sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Define physics parameters -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Add a simple box that will respond to physics -->
    <model name="physics_box">
      <pose>0 0 1 0 0 0</pose>
      <link name="box_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.083</iyy>
            <iyz>0.0</iyz>
            <izz>0.083</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## Adding a Simple Robot

Now let's create a basic humanoid robot model that will respond to physics:

```xml
<?xml version="1.0" ?>
<robot name="simple_humanoid">
  <!-- Base link -->
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
    </collision>
  </link>

  <!-- Head link -->
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
    </collision>
  </link>

  <!-- Joint connecting base to head -->
  <joint name="base_to_head" type="fixed">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 1.0"/>
  </joint>
</robot>
```

## Testing Physics Settings

To test your physics setup, launch Gazebo with your world file:

```bash
gazebo path/to/your/basic_physics_world.sdf
```

You should see the box fall due to gravity and land on the ground plane. The physics simulation should:

1. Apply gravitational acceleration (9.81 m/s² downward)
2. Handle collision detection between the box and ground
3. Apply appropriate collision response (the box stops falling)

## Physics Configuration Parameters

| Parameter | Typical Value | Purpose |
|-----------|---------------|---------|
| `gravity` | 0 0 -9.81 | Earth's gravitational acceleration |
| `max_step_size` | 0.001 | Physics simulation time step |
| `real_time_factor` | 1.0 | Simulation speed multiplier |
| `real_time_update_rate` | 1000 | Physics updates per second |

## Diagram: Gazebo Physics Architecture

```
[World Definition]
       ↓
[Physics Engine (ODE)]
       ↓
[Gravity Simulation] ← [Time Stepping]
       ↓
[Collision Detection]
       ↓
[Force Application]
       ↓
[State Updates]
```

## Test Steps

1. Create the world file with the provided XML
2. Launch Gazebo with your world file
3. Verify that objects fall due to gravity
4. Check that collisions are detected and handled properly
5. Observe the physics stability (no oscillations or instabilities)

## Summary

In this lesson, you've learned about Gazebo's physics simulation capabilities and created a basic physics world. The physics engine is fundamental to our digital twin as it provides realistic simulation of robot behavior in a virtual environment.

## Next Steps

In the next lesson, we'll explore collision detection and configuration in more detail, including how to properly configure collision shapes and materials for humanoid robots.