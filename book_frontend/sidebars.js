/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/intro',
        'module1/chapter1',
        'module1/chapter2',
        'module1/chapter3',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module2/intro',
        'module2/chapter1',
        'module2/chapter2',
        'module2/chapter3',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module3/intro',
        'module3/chapter1',
        'module3/chapter2',
        'module3/chapter3',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module4/intro',
        'module4/chapter1',
        'module4/chapter2',
        'module4/chapter3',
      ],
    },
    {
      type: 'category',
      label: 'Advanced Features',
      items: [
        'rag-integration',
        'personalization',
      ],
    },
  ],
};

module.exports = sidebars;
