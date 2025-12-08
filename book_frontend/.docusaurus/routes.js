import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'fcd'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'b42'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '44f'),
            routes: [
              {
                path: '/docs/chapter1',
                component: ComponentCreator('/docs/chapter1', '2e7'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/docs/chapter2',
                component: ComponentCreator('/docs/chapter2', '519'),
                exact: true,
                sidebar: "defaultSidebar"
              },
              {
                path: '/docs/chapter3',
                component: ComponentCreator('/docs/chapter3', '356'),
                exact: true,
                sidebar: "defaultSidebar"
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
