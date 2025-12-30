import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Build the Nervous System of Tomorrow\'s Machines',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://Muhammad-Junaid-Sajjad.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/Hackathon1/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Muhammad-Junaid-Sajjad', // Usually your GitHub org/user name.
  projectName: 'Hackathon1', // Usually your repo name.

  trailingSlash: false,

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: 'docs',
          // Enable last update
          lastVersion: 'current',
          versions: {
            current: {
              label: '2025 Edition',
            },
          },
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    // Announcement bar
    announcementBar: {
      id: 'announcement',
      content: '🚀 <b>New:</b> Complete ROS 2 Kilted Kaiju & Jetson Thor curriculum now available!',
      backgroundColor: 'var(--color-accent-blue)',
      textColor: '#ffffff',
      isCloseable: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      hideOnScroll: false,
      style: 'dark',
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          type: 'docsVersionDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/Muhammad-Junaid-Sajjad/Hackathon1',
          label: 'GitHub',
          position: 'right',
          className: 'header-github-link',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Modules',
          items: [
            {
              label: 'Module 1: The Nervous System',
              to: '/docs/M1/C1/m1-c1-s1',
            },
            {
              label: 'Module 2: The Hallucination',
              to: '/docs/M2/C1/m2-c1-s1',
            },
            {
              label: 'Module 3: The Awakening',
              to: '/docs/M3/C1/m3-c1-s1',
            },
            {
              label: 'Module 4: The Embodiment',
              to: '/docs/M4/C1/m4-c1-s1',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/Muhammad-Junaid-Sajjad/Hackathon1',
            },
            {
              label: 'Hardware Budget Guide',
              to: '/docs/hardware-budget-guide',
            },
            {
              label: 'Deployment Checklists',
              to: '/docs/deployment-checklists',
            },
          ],
        },
        {
          title: 'Reference',
          items: [
            {
              label: 'Glossary',
              to: '/docs/glossary',
            },
            {
              label: 'Self Assessment',
              to: '/docs/self-assessment',
            },
            {
              label: 'Benchmark Tables',
              to: '/docs/benchmark-tables',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics - Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['bash', 'python', 'yaml', 'json', 'cmake'],
    },
    // Algolia DocSearch configuration
    algolia: {
      appId: 'YOUR_APP_ID',
      apiKey: 'YOUR_SEARCH_API_KEY',
      indexName: 'your_index_name',
      contextualSearch: true,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
