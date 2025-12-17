import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// Server-side polyfill for localStorage/sessionStorage
const localStoragePolyfill = {
  getItem: () => null,
  setItem: () => {},
  removeItem: () => {},
  clear: () => {},
  key: () => null,
  length: 0,
  get length() {
    return 0;
  },
};

if (typeof window === 'undefined') {
  global.localStorage = localStoragePolyfill;
  global.sessionStorage = localStoragePolyfill;
}

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Building Intelligent Humanoid Robots',
  favicon: 'img/favicon.ico',
  future: { v4: true },
  url: 'https://your-vercel-project-name.vercel.app',
  baseUrl: '/',
  organizationName: 'facebook',
  projectName: 'docusaurus',
  onBrokenLinks: 'warn',
  i18n: { defaultLocale: 'en', locales: ['en'] },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: { type: ['rss', 'atom'], xslt: true },
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: { customCss: './src/css/custom.css' },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: { respectPrefersColorScheme: true },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: { alt: 'Physical AI & Humanoid Robotics Logo', src: 'img/logo.svg' },
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
            { to: '/docs/tutorial-basics/module-4-vla-integration', label: 'VLA Systems' }, // Fixed
          
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
            { label: 'ROS2', to: '/docs/module-1-ros2-fundamentals' },
            { label: 'Digital Twin', to: '/docs/module-2-digital-twin' },
            { label: 'NVIDIA Isaac', to: '/docs/module-3-isaac-ai-brain' },
            { label: 'VLA Systems', to: '/docs/tutorial-basics/module-4-vla-integration' }, // Fixed
          ],
        },
       
        {
          title: 'Community',
          items: [{ label: 'GitHub', href: 'https://github.com/facebook/docusaurus' }],
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
