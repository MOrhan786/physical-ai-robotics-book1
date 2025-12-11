---
sidebar_position: 3
description: "Learn about Unity rendering for digital twin visualization and human-robot interaction"
---

# Lesson 3: Unity Rendering and Visualization

## Overview

In this lesson, we'll explore Unity rendering for our digital twin system. Unity provides high-fidelity visualization that complements Gazebo's physics simulation, creating a complete digital twin environment for humanoid robotics.

## Understanding Unity for Digital Twins

Unity serves as the visual layer in our digital twin architecture:

- **High-quality rendering**: Realistic lighting, materials, and textures
- **Interactive environment**: Human-robot interaction capabilities
- **Real-time visualization**: Visual feedback for simulation state
- **Cross-platform support**: Runs on various hardware configurations

## Setting Up Unity Environment

First, let's create a basic Unity scene that matches our Gazebo environment:

### 1. Creating the Scene Structure

```csharp
// RobotVisualization.cs - Basic robot visualization script
using UnityEngine;

public class RobotVisualization : MonoBehaviour
{
    [Header("Robot Configuration")]
    public GameObject baseLink;
    public GameObject headLink;
    public GameObject armLink;

    [Header("ROS Connection")]
    public string robotPoseTopic = "/robot/pose";

    // Robot state
    private Vector3 targetPosition;
    private Quaternion targetRotation;

    void Start()
    {
        // Initialize robot parts
        InitializeRobotParts();
    }

    void Update()
    {
        // Smoothly interpolate to target pose
        UpdateRobotVisualization();
    }

    void InitializeRobotParts()
    {
        if (baseLink == null)
        {
            baseLink = new GameObject("BaseLink");
            baseLink.transform.SetParent(transform);
        }

        if (headLink == null)
        {
            headLink = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            headLink.name = "HeadLink";
            headLink.transform.SetParent(baseLink.transform);
            headLink.transform.localScale = new Vector3(0.3f, 0.3f, 0.3f);
            headLink.transform.localPosition = new Vector3(0, 1.0f, 0);
        }

        if (armLink == null)
        {
            armLink = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            armLink.name = "ArmLink";
            armLink.transform.SetParent(baseLink.transform);
            armLink.transform.localScale = new Vector3(0.1f, 0.2f, 0.1f);
            armLink.transform.localPosition = new Vector3(0.2f, 0.5f, 0);
            armLink.transform.localRotation = Quaternion.Euler(90, 0, 0);
        }
    }

    void UpdateRobotVisualization()
    {
        // This would be updated with ROS pose data in a real implementation
        transform.position = Vector3.Lerp(transform.position, targetPosition, Time.deltaTime * 5f);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * 5f);
    }

    public void UpdateRobotPose(Vector3 position, Quaternion rotation)
    {
        targetPosition = position;
        targetRotation = rotation;
    }
}
```

### 2. Creating the Environment

```csharp
// EnvironmentSetup.cs - Environment matching Gazebo world
using UnityEngine;

public class EnvironmentSetup : MonoBehaviour
{
    [Header("Environment Configuration")]
    public GameObject groundPlane;
    public GameObject[] obstacles;

    [Header("Lighting")]
    public Light directionalLight;

    void Start()
    {
        CreateEnvironment();
        SetupLighting();
    }

    void CreateEnvironment()
    {
        // Create ground plane (matching Gazebo's ground plane)
        if (groundPlane == null)
        {
            groundPlane = GameObject.CreatePrimitive(PrimitiveType.Plane);
            groundPlane.name = "GroundPlane";
            groundPlane.transform.position = Vector3.zero;
            groundPlane.transform.localScale = new Vector3(10, 1, 10);
        }

        // Create obstacles (matching Gazebo world)
        CreateObstacle(new Vector3(1, 0.2f, 0), new Vector3(0.4f, 0.4f, 0.4f), Color.red);
        CreateObstacle(new Vector3(-1, 0.5f, 0), new Vector3(0.4f, 1.0f, 0.4f), Color.green);
        CreateObstacle(new Vector3(0, 0.3f, 1), new Vector3(0.6f, 0.6f, 0.6f), Color.blue);
    }

    GameObject CreateObstacle(Vector3 position, Vector3 scale, Color color)
    {
        GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle.name = "Obstacle";
        obstacle.transform.position = position;
        obstacle.transform.localScale = scale;

        // Apply color
        Renderer renderer = obstacle.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = color;
        }

        return obstacle;
    }

    void SetupLighting()
    {
        if (directionalLight == null)
        {
            // Create directional light (like Gazebo's sun)
            GameObject lightObj = new GameObject("Directional Light");
            directionalLight = lightObj.AddComponent<Light>();
            directionalLight.type = LightType.Directional;
            directionalLight.color = Color.white;
            directionalLight.intensity = 1.0f;

            // Position similar to Gazebo's sun
            directionalLight.transform.position = new Vector3(10, 10, -10);
            directionalLight.transform.LookAt(Vector3.zero);
        }
    }
}
```

## Camera Setup for Unity

```csharp
// CameraController.cs - Camera for robot visualization
using UnityEngine;

public class CameraController : MonoBehaviour
{
    [Header("Camera Configuration")]
    public Transform target; // Robot to follow
    public float distance = 5.0f;
    public float height = 3.0f;
    public float smoothSpeed = 12.0f;

    [Header("Rotation")]
    public float rotationSpeed = 100.0f;
    private float currentRotationX = 0f;

    void LateUpdate()
    {
        if (target == null) return;

        // Calculate desired position
        Vector3 desiredPosition = target.position - Vector3.forward * distance + Vector3.up * height;
        Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed * Time.deltaTime);
        transform.position = smoothedPosition;

        // Look at target
        transform.LookAt(target);
    }

    void Update()
    {
        // Allow user to rotate camera with mouse
        if (Input.GetMouseButton(1)) // Right mouse button
        {
            float rotationX = Input.GetAxis("Mouse X") * rotationSpeed * Time.deltaTime;
            transform.RotateAround(target.position, Vector3.up, rotationX);
        }
    }
}
```

## Human-Robot Interaction

```csharp
// HumanRobotInteraction.cs - Basic interaction system
using UnityEngine;

public class HumanRobotInteraction : MonoBehaviour
{
    [Header("Interaction Configuration")]
    public RobotVisualization robot;
    public float moveSpeed = 5.0f;
    public float rotateSpeed = 90.0f;

    [Header("UI Elements")]
    public UnityEngine.UI.Text infoText;

    void Update()
    {
        HandleRobotControl();
        UpdateUI();
    }

    void HandleRobotControl()
    {
        if (robot == null) return;

        Vector3 movement = Vector3.zero;

        // Move forward/backward
        if (Input.GetKey(KeyCode.W))
            movement += transform.forward * moveSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.S))
            movement -= transform.forward * moveSpeed * Time.deltaTime;

        // Strafe left/right
        if (Input.GetKey(KeyCode.A))
            movement -= transform.right * moveSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.D))
            movement += transform.right * moveSpeed * Time.deltaTime;

        // Rotate left/right
        if (Input.GetKey(KeyCode.Q))
            transform.Rotate(0, -rotateSpeed * Time.deltaTime, 0);
        if (Input.GetKey(KeyCode.E))
            transform.Rotate(0, rotateSpeed * Time.deltaTime, 0);

        // Apply movement
        transform.Translate(movement, Space.World);
    }

    void UpdateUI()
    {
        if (infoText != null)
        {
            infoText.text = "Controls:\n" +
                           "WASD: Move robot\n" +
                           "Q/E: Rotate robot\n" +
                           "Right Mouse: Rotate camera";
        }
    }
}
```

## Asset Import and Configuration

To import assets from Gazebo to Unity:

1. **Export Models**: Export your Gazebo models as FBX or OBJ files
2. **Import to Unity**: Use Unity's import system
3. **Material Conversion**: Convert Gazebo materials to Unity materials
4. **Scale Adjustment**: Gazebo uses meters, Unity uses its own scale system

### Example Material Setup

```csharp
// MaterialConverter.cs - Helper for material conversion
using UnityEngine;

public class MaterialConverter : MonoBehaviour
{
    public static Material CreateMaterialFromGazeboProperties(
        Color ambient, Color diffuse, Color specular, float shininess)
    {
        Material material = new Material(Shader.Find("Standard"));

        material.SetColor("_Color", diffuse);
        material.SetColor("_EmissionColor", ambient);
        material.SetFloat("_Metallic", 0.0f); // Convert specular to metallic if needed
        material.SetFloat("_Glossiness", shininess);

        return material;
    }
}
```

## Comparison: Gazebo vs Unity Visualization

| Aspect | Gazebo | Unity |
|--------|--------|-------|
| **Purpose** | Physics simulation | High-fidelity rendering |
| **Realism** | Moderate | High |
| **Performance** | Optimized for physics | Optimized for graphics |
| **Interactivity** | Limited | Extensive |
| **Customization** | Through SDF/URDF | Through scripts and materials |

## Diagram: Unity-Gazebo Integration

```
[Gazebo Physics] → [ROS Bridge] → [Unity Visualization]
       ↓               ↓                ↓
[State Updates] → [Data Transfer] → [Visual Updates]
```

## Test Steps

1. Create a new Unity project
2. Implement the basic robot visualization script
3. Set up the environment matching Gazebo
4. Add camera and interaction controls
5. Test the visualization system
6. Verify that the Unity scene matches the Gazebo environment

## Performance Optimization Tips

1. **LOD System**: Use Level of Detail for complex models
2. **Occlusion Culling**: Hide objects not visible to camera
3. **Texture Compression**: Optimize textures for performance
4. **Object Pooling**: Reuse objects instead of creating/destroying
5. **Batching**: Combine similar objects for rendering efficiency

## Summary

In this lesson, you've learned how to set up Unity for digital twin visualization, including environment creation, robot visualization, camera setup, and human-robot interaction. Unity provides the high-fidelity rendering layer that complements Gazebo's physics simulation.

## Next Steps

In the next lesson, we'll explore sensor simulation in Gazebo and how to visualize sensor data in our Unity environment.