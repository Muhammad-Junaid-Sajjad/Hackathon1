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
  textbookSidebar: [
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      collapsible: true,
      collapsed: false,
      items: [
        'intro/m1-gateway',
        {
          type: 'category',
          label: 'Chapter 1: Foundations & Hardware',
          items: [
            'M1/C1/m1-c1-s1',
            'M1/C1/m1-c1-s2',
            'M1/C1/m1-c1-s3',
            'M1/C1/m1-c1-s4',
            'M1/C1/m1-c1-s5',
            'M1/C1/m1-c1-s6',
            'M1/C1/m1-c1-s7',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: ROS 2 Logic Layer',
          items: [
            'M1/C2/m1-c2-s1',
            'M1/C2/m1-c2-s2',
            'M1/C2/m1-c2-s3',
            'M1/C2/m1-c2-s4',
            'M1/C2/m1-c2-s5',
            'M1/C2/m1-c2-s6',
            'M1/C2/m1-c2-s7',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Humanoid URDF & TF2',
          items: [
            'M1/C3/m1-c3-s1',
            'M1/C3/m1-c3-s2',
            'M1/C3/m1-c3-s3',
            'M1/C3/m1-c3-s4',
            'M1/C3/m1-c3-s5',
            'M1/C3/m1-c3-s6',
            'M1/C3/m1-c3-s7',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation',
      collapsible: true,
      collapsed: true,
      items: [
        'intro/m2-gateway',
        {
          type: 'category',
          label: 'Chapter 1: Gazebo Physics Engine',
          items: [
            'M2/C1/m2-c1-s1',
            'M2/C1/m2-c1-s2',
            'M2/C1/m2-c1-s3',
            'M2/C1/m2-c1-s4',
            'M2/C1/m2-c1-s5',
            'M2/C1/m2-c1-s6',
            'M2/C1/m2-c1-s7',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Sensor Simulation',
          items: [
            'M2/C2/m2-c2-s1',
            'M2/C2/m2-c2-s2',
            'M2/C2/m2-c2-s3',
            'M2/C2/m2-c2-s4',
            'M2/C2/m2-c2-s5',
            'M2/C2/m2-c2-s6',
            'M2/C2/m2-c2-s7',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Unity & Human-Robot Interaction',
          items: [
            'M2/C3/m2-c3-s1',
            'M2/C3/m2-c3-s2',
            'M2/C3/m2-c3-s3',
            'M2/C3/m2-c3-s4',
            'M2/C3/m2-c3-s5',
            'M2/C3/m2-c3-s6',
            'M2/C3/m2-c3-s7',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac & Sim-to-Real',
      collapsible: true,
      collapsed: true,
      items: [
        'intro/m3-gateway',
        {
          type: 'category',
          label: 'Chapter 1: Isaac Sim & Omniverse',
          items: [
            'M3/C1/m3-c1-s1',
            'M3/C1/m3-c1-s2',
            'M3/C1/m3-c1-s3',
            'M3/C1/m3-c1-s4',
            'M3/C1/m3-c1-s5',
            'M3/C1/m3-c1-s6',
            'M3/C1/m3-c1-s7',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Isaac ROS Perception',
          items: [
            'M3/C2/m3-c2-s1',
            'M3/C2/m3-c2-s2',
            'M3/C2/m3-c2-s3',
            'M3/C2/m3-c2-s4',
            'M3/C2/m3-c2-s5',
            'M3/C2/m3-c2-s6',
            'M3/C2/m3-c2-s7',
            'M3/C2/m3-c2-s8',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Reinforcement Learning & Deployment',
          items: [
            'M3/C3/m3-c3-s1',
            'M3/C3/m3-c3-s2',
            'M3/C3/m3-c3-s3',
            'M3/C3/m3-c3-s4',
            'M3/C3/m3-c3-s5',
            'M3/C3/m3-c3-s6',
            'M3/C3/m3-c3-s7',
            'M3/C3/m3-c3-s8',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Integration',
      collapsible: true,
      collapsed: true,
      items: [
        'intro/m4-gateway',
        {
          type: 'category',
          label: 'Chapter 1: Multimodal Perception',
          items: [
            'M4/C1/m4-c1-s1',
            'M4/C1/m4-c1-s2',
            'M4/C1/m4-c1-s3',
            'M4/C1/m4-c1-s4',
            'M4/C1/m4-c1-s5',
            'M4/C1/m4-c1-s6',
            'M4/C1/m4-c1-s7',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Language-Driven Planning',
          items: [
            'M4/C2/m4-c2-s1',
            'M4/C2/m4-c2-s2',
            'M4/C2/m4-c2-s3',
            'M4/C2/m4-c2-s4',
            'M4/C2/m4-c2-s5',
            'M4/C2/m4-c2-s6',
            'M4/C2/m4-c2-s7',
            'M4/C2/m4-c2-s8',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Capstone Integration',
          items: [
            'M4/C3/m4-c3-s1',
            'M4/C3/m4-c3-s2',
            'M4/C3/m4-c3-s3',
            'M4/C3/m4-c3-s4',
            'M4/C3/m4-c3-s5',
            'M4/C3/m4-c3-s6',
            'M4/C3/m4-c3-s7',
          ],
        },
        'intro/final-verdict',
      ],
    },
    {
      type: 'category',
      label: '📚 Reference Materials',
      collapsible: true,
      collapsed: true,
      items: [
        'glossary',
        'references',
        'benchmark-tables',
        'hardware-budget-guide',
        'deployment-checklists',
        'certification-pathways',
        'self-assessment',
      ],
    },
  ],
};

export default sidebars;
