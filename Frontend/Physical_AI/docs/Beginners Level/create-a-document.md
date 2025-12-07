---
sidebar_position: 2
---

# Creating Physical AI Documentation

Documentation in Physical AI and Robotics projects is **essential for maintaining**:

- Robot model specifications
- Simulation world descriptions
- AI behavior implementations
- Integration procedures

## Document your first Robot Model

Create a specification document at `docs/robots/my_robot_spec.md`:

```md title="docs/robots/my_robot_spec.md"
# My Robot Specification

This document describes the physical and functional properties of my robot.
```

A new specification document is now available in your documentation system.

## Structure your Robot Documentation

Organize your documentation with proper metadata to create a logical structure:

```md title="docs/robots/my_robot_spec.md" {1-4}
---
sidebar_label: 'My Robot Specs'
sidebar_position: 3
---

# My Robot Specification

This document describes the physical and functional properties of my robot.
```

You can also organize your documentation in the `sidebars.ts` file:

```ts title="sidebars.ts"
export default {
  tutorialSidebar: [
    'intro',
    // highlight-next-line
    'robots/my_robot_spec',
    {
      type: 'category',
      label: 'Robot Specifications',
      items: ['beginners-level/create-a-document'],
    },
  ],
};
```
