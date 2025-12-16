import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

// Create localStorage polyfill for server-side rendering
const localStoragePolyfill = {
  getItem: () => null,
  setItem: () => {},
  removeItem: () => {},
  clear: () => {},
  key: () => null,
  length: 0,
  get length() {
    return 0;
  }
};

// Apply the polyfill in Node.js environment
if (typeof window === 'undefined') {
  global.localStorage = localStoragePolyfill;
  global.sessionStorage = localStoragePolyfill;
}

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Building Intelligent Humanoid Robots',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://your-vercel-project-name.vercel.app', // Replace with your actual Vercel URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'facebook', // Usually your GitHub org/user name.
  projectName: 'docusaurus', // Usually your repo name.

  onBrokenLinks: 'warn',

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
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
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
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        { to: '/', label: 'Home', position: 'left' },
        {
          type: 'dropdown',
          label: 'Modules',
          position: 'left',
          items: [
            { to: '/docs/module-1-ros2-fundamentals', label: 'ROS 2' },
            { to: '/docs/module-2-digital-twin', label: 'Digital Twin' },
            { to: '/docs/module-3-isaac-ai-brain', label: 'NVIDIA Isaac' },
            { to: '/docs/tutorial-basics/module-4-vla-integration', label: 'VLA Systems' },
            { to: '/docs/capstone', label: 'Capstone' },
          ],
        },
        {
          href: 'https://github.com/facebook/docusaurus',
          label: 'GitHub',
          position: 'right',
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
              label: 'ROS2',
              to: '/docs/module-1-ros2-fundamentals',
            },
            {
              label: 'Digital Twin',
              to: '/docs/module-2-digital-twin',
            },
            {
              label: 'NVIDIA Isaac',
              to: '/docs/module-3-isaac-ai-brain',
            },
            {
              label: 'VLA Systems',
              to: '/docs/tutorial-basics/module-4-vla-integration',
            },
          ],
        },
        {
          title: 'Project',
          items: [
            {
              label: 'Capstone',
              to: '/docs/capstone',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/facebook/docusaurus',
            },
          ],
        },
      ],
      copyright: `© 2025 Physical AI & Humanoid Robotics — Built with Docusaurus & Spec-Kit Plus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;