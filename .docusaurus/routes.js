import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'eda'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'dcf'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'ee7'),
            routes: [
              {
                path: '/docs/',
                component: ComponentCreator('/docs/', '698'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/appendices/hardware-guide',
                component: ComponentCreator('/docs/appendices/hardware-guide', 'd50'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/appendices/weekly-roadmap',
                component: ComponentCreator('/docs/appendices/weekly-roadmap', '46c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/capstone/main',
                component: ComponentCreator('/docs/capstone/main', '5ff'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/foundations/intro',
                component: ComponentCreator('/docs/foundations/intro', '872'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/rclpy',
                component: ComponentCreator('/docs/module1/rclpy', '61a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/ros2-actions',
                component: ComponentCreator('/docs/module1/ros2-actions', '79d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/ros2-nodes',
                component: ComponentCreator('/docs/module1/ros2-nodes', '08d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/ros2-services',
                component: ComponentCreator('/docs/module1/ros2-services', '7f7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/ros2-topics',
                component: ComponentCreator('/docs/module1/ros2-topics', '3f3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/gazebo-simulation',
                component: ComponentCreator('/docs/module2/gazebo-simulation', '954'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/unity-rendering',
                component: ComponentCreator('/docs/module2/unity-rendering', 'dec'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/urdf-modeling',
                component: ComponentCreator('/docs/module2/urdf-modeling', 'e34'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/isaac-ros',
                component: ComponentCreator('/docs/module3/isaac-ros', 'd4e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/isaac-sim-basics',
                component: ComponentCreator('/docs/module3/isaac-sim-basics', 'a1c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/nav2-biped',
                component: ComponentCreator('/docs/module3/nav2-biped', '8a6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/llm-planning',
                component: ComponentCreator('/docs/module4/llm-planning', 'e9b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/vla-pipelines',
                component: ComponentCreator('/docs/module4/vla-pipelines', '44e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/whisper',
                component: ComponentCreator('/docs/module4/whisper', 'ae6'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
