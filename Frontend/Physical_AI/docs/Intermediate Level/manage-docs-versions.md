---
sidebar_position: 1
---

# Managing Physical AI Project Versions

Physical AI and Robotics projects often require managing multiple versions of robot models, simulation environments, and AI algorithms.

## Version your robotics documentation

Maintain different versions of your robot specifications and AI implementations:

```bash
npm run docusaurus docs:version robot-v1.0
```

The `docs` folder is copied into `versioned_docs/version-robot-v1.0` and `versions.json` is created.

Your robotics documentation now has 2 versions:

- `robot-v1.0` at `http://localhost:3000/docs/` for the version 1.0 robot specs
- `current` at `http://localhost:3000/docs/next/` for the **upcoming, unreleased robot designs**

## Add a Version Dropdown for Robotics Projects

To navigate seamlessly across robot versions, add a version dropdown.

Modify the `docusaurus.config.js` file:

```js title="docusaurus.config.js"
export default {
  themeConfig: {
    navbar: {
      items: [
        // highlight-start
        {
          type: 'docsVersionDropdown',
        },
        // highlight-end
      ],
    },
  },
};
```

The docs version dropdown appears in your navbar, allowing easy navigation between different robot model versions.

## Update existing robot specifications

It is possible to edit versioned robotics docs in their respective folder:

- `versioned_docs/version-robot-v1.0/robot-specs.md` updates `http://localhost:3000/docs/robot-specs`
- `docs/robot-specs.md` updates `http://localhost:3000/docs/next/robot-specs`

This is crucial for maintaining historical records of robot designs and AI algorithm improvements.
