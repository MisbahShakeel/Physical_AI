import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI And Robotics Learning',
  tagline: 'Mastering the intersection of Artificial Intelligence and Robotics',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://physical-ai.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'MisbahShakeel', // Usually your GitHub org/user name.
  projectName: 'Physical_AI', // Usually your repo name.

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
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/MisbahShakeel/Physical_AI/tree/main/Frontend/Physical_AI/',
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
            'https://github.com/MisbahShakeel/Physical_AI/tree/main/Frontend/Physical_AI/',
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
    image: 'img/robot-social-card.svg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI And Robotics Learning',
      logo: {
        alt: 'Physical AI And Robotics Learning Logo',
        src: 'img/robot-logo.svg',
      },
      items: [
        {
          type: 'doc',
          docId: 'Physical AI And Robotics Learning/intro',
          position: 'left',
          label: 'Tutorial',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
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
          title: 'Learning Resources',
          items: [
            {
              label: 'Getting Started',
              to: '/docs/Physical AI And Robotics Learning/intro',
            },
            {
              label: 'Beginner Tutorials',
              to: '/docs/Beginners Level/create-a-page',
            },
            {
              label: 'Advanced Topics',
              to: '/docs/Intermediate Level/manage-docs-versions',
            },
          ],
        },
        {
          title: 'Robotics Community',
          items: [
            {
              label: 'ROS Answers',
              href: 'https://answers.ros.org/',
            },
            {
              label: 'Robotics Stack Exchange',
              href: 'https://robotics.stackexchange.com/',
            },
            {
              label: 'Gazebo Simulation',
              href: 'https://gazebosim.org/',
            },
          ],
        },
        {
          title: 'Projects & Research',
          items: [
            {
              label: 'Research Papers',
              href: 'https://scholar.google.com/scholar?q=physical+ai+robotics',
            },
            {
              label: 'GitHub Repository',
              href: 'https://github.com/physical-ai/physical-ai',
            },
            {
              label: 'Open Source Robotics',
              href: 'https://www.osrfoundation.org/',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI And Robotics Learning. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
