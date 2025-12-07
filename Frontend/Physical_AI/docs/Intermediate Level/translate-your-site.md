---
sidebar_position: 2
---

# Collaborate on Physical AI Projects

Let's set up collaboration for your Physical AI and Robotics documentation to work with international teams.

## Configure multi-team support

Modify `docusaurus.config.js` to organize documentation for different research teams:

```js title="docusaurus.config.js"
export default {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'zh', 'de'], // Support for international robotics teams
  },
};
```

## Organize documentation for teams

Create team-specific documentation folders:

```bash
mkdir -p docs/teams/perception-team/
mkdir -p docs/teams/navigation-team/
mkdir -p docs/teams/manipulation-team/

cp docs/intermediate-level/advanced-simulation.md docs/teams/perception-team/simulation-guide.md
```

Each team can maintain their own documentation for robot perception, navigation, and manipulation modules.

## Start your collaborative site

Start your documentation site with team-specific views:

```bash
npm run start
```

Your collaborative site is accessible at [http://localhost:3000/](http://localhost:3000/) where different robotics teams can access their specialized documentation.

:::tip Collaboration Tip

Set up different documentation sections for each robotics subsystem team.

:::

## Add a Team Navigation

To navigate seamlessly across team documentation, add a team dropdown.

Modify the `docusaurus.config.js` file:

```js title="docusaurus.config.js"
export default {
  themeConfig: {
    navbar: {
      items: [
        // highlight-start
        {
          type: 'localeDropdown',
        },
        // highlight-end
      ],
    },
  },
};
```

The locale dropdown now appears in your navbar, allowing international robotics teams to access documentation in their preferred language.

## Build your collaborative documentation

Build your site for a specific team/locale:

```bash
npm run build -- --locale zh
```

Or build your complete site to include all team documentation at once:

```bash
npm run build
```
