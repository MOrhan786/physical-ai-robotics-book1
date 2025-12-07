# Lesson 5: Simplifying with ROS2 Launch Files

### Learning Objectives

*   Understand what a ROS2 launch file is.
*   Learn how to write a simple launch file in Python.
*   Use a launch file to start multiple nodes at once.

### What are Launch Files?

Imagine you have a project with many different ROS2 nodes. To start your robot, you might need to open several terminals and run a `ros2 run` command in each one. This is slow and it's easy to forget a node.

A **launch file** solves this problem! It's a single script that can start and configure all your nodes at once. You run one command, and your entire ROS2 application starts up correctly.

### Example: Python Launch File

ROS2 launch files are typically written in Python. They are simple to create. Let's make a launch file that starts the turtle simulator and the keyboard controller.

Here is an example `my_first_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop'
        )
    ])
```

This script imports the necessary tools, then defines a function called `generate_launch_description`. This function describes two nodes we want to run: `turtlesim_node` and `turtle_teleop_key`.

### How to Run It

To run this launch file, you would save it inside a ROS2 package (e.g., in a `launch/` directory). Then, you use the `ros2 launch` command:

`ros2 launch <your_package_name> my_first_launch.py`

When you run this, you will see both the turtlesim window and the terminal waiting for keyboard input to control the turtle. You only had to run one command!

### Diagram Description

Imagine a simple diagram. At the top, there is a box labeled "Terminal" with the `ros2 launch` command inside. An arrow points from this box to a central box labeled "Python Launch File". From this central box, two arrows point outwards to two final boxes: one labeled "Turtlesim Node" and another labeled "Keyboard Control Node". This shows how one command starts everything.

### Short Exercise

1.  Create a new launch file called `my_exercise_launch.py`.
2.  Modify the file to launch only the `turtlesim_node`.
3.  Can you add another node from a previous lesson (like a simple publisher or subscriber) to this launch file?

### Summary

You've learned that launch files are a powerful way to manage complex ROS2 systems. They let you start and configure multiple nodes with a single command, making your workflow much more efficient. From now on, you should use launch files for all your projects.
