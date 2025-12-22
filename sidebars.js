// @ts-check

/**
 * @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  bookSidebar: [

    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'doc',
      id: 'chatbot',
      label: 'AI Assistant',
    },
    {
      type: 'category',
      label: 'Module 1: Introduction to Physical AI',
      collapsible: true,
      collapsed: false,
      items: [
        'Module 1 - introduction-to-physical-ai/foundations-of-physical-ai-and-embodied-intelligence',
        'Module 1 - introduction-to-physical-ai/humanoid-robotics-landscape',
        'Module 1 - introduction-to-physical-ai/sensor-systems-integration',
      ],
    },

    {
      type: 'category',
      label: 'Module 2: Robotic Nervous System',
      collapsible: true,
      collapsed: true,
      items: [
        'Module 2 - Robotic-Nervous-System/robotic-nervous-system',
        'Module 2 - Robotic-Nervous-System/architecture',
        'Module 2 - Robotic-Nervous-System/installation',
        'Module 2 - Robotic-Nervous-System/first-node',
        'Module 2 - Robotic-Nervous-System/urdf-intro',
        'Module 2 - Robotic-Nervous-System/services',
        'Module 2 - Robotic-Nervous-System/topics',
        'Module 2 - Robotic-Nervous-System/exercises',
      ],
    },

    {
      type: 'category',
      label: 'Module 3: Digital Twin',
      collapsible: true,
      collapsed: true,
      items: [
        'Module 3 -Digital Twin/prerequisites-setup',
        'Module 3 -Digital Twin/introduction-to-digital-twins',
        'Module 3 -Digital Twin/gazebo-physics-simulation',
        'Module 3 -Digital Twin/sensor-simulation',
        'Module 3 -Digital Twin/unity-integration',
        'Module 3 -Digital Twin/advanced-digital-twin-workflows',
      ],
    },

    {
      type: 'category',
      label: 'Module 4: The AI Robot Brain (NVIDIA Isaac)',
      collapsible: true,
      collapsed: true,
      items: [
        'Module_4_-_The_AI_Robot_Brain_NVIDIA_Isaac/prerequisites',
        'Module_4_-_The_AI_Robot_Brain_NVIDIA_Isaac/introduction-to-isaac-ecosystem',
        'Module_4_-_The_AI_Robot_Brain_NVIDIA_Isaac/isaac-sim-fundamentals',
        'Module_4_-_The_AI_Robot_Brain_NVIDIA_Isaac/vslam-with-isaac',
        'Module_4_-_The_AI_Robot_Brain_NVIDIA_Isaac/nav2-integration',
        'Module_4_-_The_AI_Robot_Brain_NVIDIA_Isaac/synthetic-data-generation',
        'Module_4_-_The_AI_Robot_Brain_NVIDIA_Isaac/end-to-end-workflows',
        'Module_4_-_The_AI_Robot_Brain_NVIDIA_Isaac/quickstart-guide',
      ],
    },
  ],
};

export default sidebars;
