import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 0: Introduction to Robotics',
      items: [
        'intro-to-robotics/lesson-1-basics',
        'intro-to-robotics/lesson-2-components',
        'intro-to-robotics/lesson-3-control'
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Robotics Module',
      items: [
        'ros2-robotics-module/lesson1-nodes',
        'ros2-robotics-module/lesson2-topics',
        'ros2-robotics-module/lesson3-services',
        'ros2-robotics-module/lesson4-actions',
        'ros2-robotics-module/lesson5-launch-files'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation',
      items: [
        'digital-twin-sim/lesson1-gazebo-physics',
        'digital-twin-sim/lesson2-collisions',
        'digital-twin-sim/lesson3-unity-rendering',
        'digital-twin-sim/lesson4-sensor-simulation'
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
