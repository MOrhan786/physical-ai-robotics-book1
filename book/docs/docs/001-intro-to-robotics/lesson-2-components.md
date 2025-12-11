---
title: "Robot Components and Architecture"
description: "Understanding the building blocks of robotic systems"
sidebar_label: "Lesson 2: Components"
---

# Lesson 2: Robot Components and Architecture

<!-- ![Robot Components Architecture](/img/robot-components-architecture.png) -->

*Figure 2: Architecture of robot components showing mechanical, electrical, and sensing systems.*

> **Note**: In a complete implementation, this figure would show a diagram of the architecture of robot components showing mechanical, electrical, and sensing systems.

## Mechanical Components

The mechanical structure of a robot provides the physical framework and enables movement. Key mechanical components include:

### Joints and Degrees of Freedom
- **Revolute Joints**: Allow rotational movement around a single axis
- **Prismatic Joints**: Enable linear sliding motion
- **Spherical Joints**: Permit rotation in multiple directions
- **Degrees of Freedom (DOF)**: The number of independent movements a robot can perform

### Links and End Effectors
- **Links**: Rigid components that connect joints
- **End Effectors**: Tools or devices at the end of robotic arms (grippers, welding tools, cameras)

## Electrical Components

### Power Systems
- **Batteries**: Rechargeable power sources (Li-ion, NiMH, etc.)
- **Power Management**: Voltage regulation and distribution
- **Wiring Harnesses**: Organized cable systems connecting components

### Control Electronics
- **Microcontrollers**: Small computers for real-time control (Arduino, Raspberry Pi, etc.)
- **Single Board Computers**: More powerful computing platforms (Raspberry Pi 4, NVIDIA Jetson)
- **Motor Controllers**: Specialized circuits for driving motors

## Sensing Systems

### Internal Sensors
- **Encoders**: Measure joint positions and motor rotation
- **IMUs**: Measure orientation, velocity, and gravitational forces
- **Force/Torque Sensors**: Detect interaction forces

### External Sensors
- **Cameras**: Visual perception for object recognition and navigation
- **LIDAR**: Light Detection and Ranging for distance measurement
- **Ultrasonic Sensors**: Sound-based distance measurement
- **Infrared Sensors**: Detect objects and measure distance

## Communication Systems

Modern robots often incorporate various communication methods:
- **Wired Communication**: USB, Ethernet, CAN bus
- **Wireless Communication**: WiFi, Bluetooth, Zigbee
- **Inter-Process Communication**: For multi-component systems

## Control Architecture

### Centralized vs. Distributed Control
- **Centralized**: Single controller manages all functions
- **Distributed**: Multiple controllers coordinate tasks

### Control Loops
- **Open Loop**: Commands sent without feedback
- **Closed Loop**: Uses sensor feedback to adjust behavior

## Integration Considerations

When designing robotic systems, engineers must consider (these integration aspects are essential for the control systems we'll discuss in [Lesson 3: Robot Control Systems](../intro-to-robotics/lesson-3-control)):
- **Mechanical Tolerances**: Ensuring components fit and function together
- **Electrical Compatibility**: Voltage, current, and signal level matching
- **Thermal Management**: Heat dissipation across components
- **Weight Distribution**: Maintaining stability and mobility

## Next Steps

In the next lesson, we'll explore robot control systems and how these components work together to achieve desired behaviors.

[Previous: Lesson 1 - Robotics Basics](../intro-to-robotics/lesson-1-basics) | [Next: Lesson 3 - Robot Control Systems](../intro-to-robotics/lesson-3-control)