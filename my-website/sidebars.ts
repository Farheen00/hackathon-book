import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Tutorial',
      items: [
        'tutorial-basics/create-a-document',
        'tutorial-basics/create-a-page',
        'tutorial-basics/deploy-your-site',
      ],
    },
    {
      type: 'category',
      label: 'Module 1 – The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2-fundamentals/index',
        {
          type: 'category',
          label: 'Chapter 1: ROS 2 Fundamentals',
          items: [
            'module-1-ros2-fundamentals/chapter-1-fundamentals/index',
            'module-1-ros2-fundamentals/chapter-1-fundamentals/nodes',
            'module-1-ros2-fundamentals/chapter-1-fundamentals/topics',
            'module-1-ros2-fundamentals/chapter-1-fundamentals/services',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Python-to-ROS Control Bridge',
          items: [
            'module-1-ros2-fundamentals/chapter-2-python-bridge/index',
            'module-1-ros2-fundamentals/chapter-2-python-bridge/rclpy-intro',
            'module-1-ros2-fundamentals/chapter-2-python-bridge/publisher-subscriber',
            'module-1-ros2-fundamentals/chapter-2-python-bridge/ai-robot-integration',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Humanoid Robot URDF',
          items: [
            'module-1-ros2-fundamentals/chapter-3-urdf-modeling/index',
            'module-1-ros2-fundamentals/chapter-3-urdf-modeling/urdf-structure',
            'module-1-ros2-fundamentals/chapter-3-urdf-modeling/humanoid-links',
            'module-1-ros2-fundamentals/chapter-3-urdf-modeling/urdf-validation',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2 – The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/index',
        {
          type: 'category',
          label: 'Chapter 1: Gazebo Physics Essentials',
          items: [
            'module-2-digital-twin/chapter-1-gazebo-physics/index',
            'module-2-digital-twin/chapter-1-gazebo-physics/ch2-s1-physics-worlds',
            'module-2-digital-twin/chapter-1-gazebo-physics/ch2-s2-collision-detection',
            'module-2-digital-twin/chapter-1-gazebo-physics/ch2-s3-robot-dynamics',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Unity for Humanoid Interaction',
          items: [
            'module-2-digital-twin/chapter-2-unity-environments/index',
            'module-2-digital-twin/chapter-2-unity-environments/ch2-s4-scene-design',
            'module-2-digital-twin/chapter-2-unity-environments/ch2-s5-lighting-materials',
            'module-2-digital-twin/chapter-2-unity-environments/ch2-s6-humanoid-interaction',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Simulated Sensors',
          items: [
            'module-2-digital-twin/chapter-3-sensor-simulation/index',
            'module-2-digital-twin/chapter-3-sensor-simulation/ch2-s7-lidar-simulation',
            'module-2-digital-twin/chapter-3-sensor-simulation/ch2-s8-depth-camera-simulation',
            'module-2-digital-twin/chapter-3-sensor-simulation/ch2-s9-imu-simulation',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3 – The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3-isaac-ai-brain/index',
        {
          type: 'category',
          label: 'Chapter 1: Isaac Sim - Photorealistic Simulation & Synthetic Data',
          items: [
            'module-3-isaac-ai-brain/chapter-1-isaac-sim/ch3-isaac-sim',
            'module-3-isaac-ai-brain/chapter-1-isaac-sim/ch3-s1-isaac-envs',
            'module-3-isaac-ai-brain/chapter-1-isaac-sim/ch3-s2-synthetic-data',
            'module-3-isaac-ai-brain/chapter-1-isaac-sim/ch3-s3-domain-transfer',
            'module-3-isaac-ai-brain/chapter-1-isaac-sim/ch3-s10-troubleshooting',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Isaac ROS - VSLAM and Hardware-Accelerated Perception',
          items: [
            'module-3-isaac-ai-brain/chapter-2-isaac-ros/ch3-isaac-ros',
            'module-3-isaac-ai-brain/chapter-2-isaac-ros/ch3-s4-vslam-concepts',
            'module-3-isaac-ai-brain/chapter-2-isaac-ros/ch3-s5-hardware-acceleration',
            'module-3-isaac-ai-brain/chapter-2-isaac-ros/ch3-s6-perception-pipelines',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Nav2 Path Planning - Bipedal Humanoid Navigation',
          items: [
            'module-3-isaac-ai-brain/chapter-3-nav2-path-planning/ch3-nav2-path-planning',
            'module-3-isaac-ai-brain/chapter-3-nav2-path-planning/ch3-s7-bipedal-navigation',
            'module-3-isaac-ai-brain/chapter-3-nav2-path-planning/ch3-s8-path-planning-constraints',
            'module-3-isaac-ai-brain/chapter-3-nav2-path-planning/ch3-s9-humanoid-locomotion',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4 – Vision-Language-Action (VLA)',
      items: [
        'tutorial-basics/module-4-vla-integration/index',
        {
          type: 'category',
          label: 'Chapter 1: Voice-to-Action - Using OpenAI Whisper for Command Recognition',
          items: [
            'tutorial-basics/module-4-vla-integration/chapter-1-whisper-v2a/ch4-s1-whisper-integration',
            'tutorial-basics/module-4-vla-integration/chapter-1-whisper-v2a/ch4-s2-command-recognition',
            'tutorial-basics/module-4-vla-integration/chapter-1-whisper-v2a/ch4-s3-v2a-pipeline',
            'tutorial-basics/module-4-vla-integration/chapter-1-whisper-v2a/ch4-whisper-v2a',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Cognitive Planning - Converting Natural Language to ROS 2 Action Sequences',
          items: [
            'tutorial-basics/module-4-vla-integration/chapter-2-cognitive-planning/ch4-cognitive-planning',
            'tutorial-basics/module-4-vla-integration/chapter-2-cognitive-planning/ch4-s4-nlp-processing',
            'tutorial-basics/module-4-vla-integration/chapter-2-cognitive-planning/ch4-s5-action-translation',
            'tutorial-basics/module-4-vla-integration/chapter-2-cognitive-planning/ch4-s6-planning-algorithms',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Capstone Integration - Autonomous Humanoid Task Execution',
          items: [
            'tutorial-basics/module-4-vla-integration/chapter-3-capstone-integration/ch4-capstone-integration',
            'tutorial-basics/module-4-vla-integration/chapter-3-capstone-integration/ch4-s7-system-integration',
            'tutorial-basics/module-4-vla-integration/chapter-3-capstone-integration/ch4-s8-task-execution',
            'tutorial-basics/module-4-vla-integration/chapter-3-capstone-integration/ch4-s9-validation-scenarios',
          ],
        },
      ],
    },
  ],
};

export default sidebars;
