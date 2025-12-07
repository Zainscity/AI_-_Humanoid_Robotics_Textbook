import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/humanoid_robotics_book/docs',
    component: ComponentCreator('/humanoid_robotics_book/docs', '2c6'),
    routes: [
      {
        path: '/humanoid_robotics_book/docs',
        component: ComponentCreator('/humanoid_robotics_book/docs', '06d'),
        routes: [
          {
            path: '/humanoid_robotics_book/docs',
            component: ComponentCreator('/humanoid_robotics_book/docs', '063'),
            routes: [
              {
                path: '/humanoid_robotics_book/docs/',
                component: ComponentCreator('/humanoid_robotics_book/docs/', '82e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/appendices/hardware-guide',
                component: ComponentCreator('/humanoid_robotics_book/docs/appendices/hardware-guide', '655'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/appendices/weekly-roadmap',
                component: ComponentCreator('/humanoid_robotics_book/docs/appendices/weekly-roadmap', '9df'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/capstone/main',
                component: ComponentCreator('/humanoid_robotics_book/docs/capstone/main', '24f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/foundations/intro',
                component: ComponentCreator('/humanoid_robotics_book/docs/foundations/intro', '266'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/module1/rclpy',
                component: ComponentCreator('/humanoid_robotics_book/docs/module1/rclpy', 'fc3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/module1/ros2-actions',
                component: ComponentCreator('/humanoid_robotics_book/docs/module1/ros2-actions', 'a1e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/module1/ros2-nodes',
                component: ComponentCreator('/humanoid_robotics_book/docs/module1/ros2-nodes', '2d0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/module1/ros2-services',
                component: ComponentCreator('/humanoid_robotics_book/docs/module1/ros2-services', 'e30'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/module1/ros2-topics',
                component: ComponentCreator('/humanoid_robotics_book/docs/module1/ros2-topics', '38a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/module2/gazebo-simulation',
                component: ComponentCreator('/humanoid_robotics_book/docs/module2/gazebo-simulation', 'cae'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/module2/unity-rendering',
                component: ComponentCreator('/humanoid_robotics_book/docs/module2/unity-rendering', 'b60'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/module2/urdf-modeling',
                component: ComponentCreator('/humanoid_robotics_book/docs/module2/urdf-modeling', 'b9f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/module3/isaac-ros',
                component: ComponentCreator('/humanoid_robotics_book/docs/module3/isaac-ros', '46c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/module3/isaac-sim-basics',
                component: ComponentCreator('/humanoid_robotics_book/docs/module3/isaac-sim-basics', '4bc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/module3/nav2-biped',
                component: ComponentCreator('/humanoid_robotics_book/docs/module3/nav2-biped', '035'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/module4/llm-planning',
                component: ComponentCreator('/humanoid_robotics_book/docs/module4/llm-planning', '0d8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/module4/vla-pipelines',
                component: ComponentCreator('/humanoid_robotics_book/docs/module4/vla-pipelines', 'c57'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/humanoid_robotics_book/docs/module4/whisper',
                component: ComponentCreator('/humanoid_robotics_book/docs/module4/whisper', '05e'),
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
    path: '/humanoid_robotics_book/',
    component: ComponentCreator('/humanoid_robotics_book/', 'cd5'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
