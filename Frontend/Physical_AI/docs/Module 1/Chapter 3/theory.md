---
title: "Understanding URDF (Unified Robot Description Format) for Humanoids"
---

## Learning Objectives

By the end of this chapter, you will be able to:
- Define and explain the purpose of URDF (Unified Robot Description Format) in robotics
- Create and structure URDF files for simple and complex robot models
- Understand the XML syntax and structure of URDF files
- Define robot links, joints, and their properties in URDF
- Implement visual and collision properties for robot components
- Apply geometric transformations and coordinate systems in URDF
- Create URDF files specifically for humanoid robot structures
- Validate and debug URDF files for correctness
- Integrate URDF models with ROS 2 and simulation environments

## Core Theory

### Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used to describe robots in the ROS ecosystem. It provides a complete definition of a robot's physical structure, including its kinematic and dynamic properties. URDF is fundamental to robot simulation, visualization, and control in ROS-based systems.

URDF serves as the standard format for robot description in ROS and is used by various tools including RViz for visualization, Gazebo for simulation, MoveIt! for motion planning, and the robot_state_publisher for transforming joint states to coordinate frames.

### URDF Structure and XML Syntax

URDF files are XML documents with a specific structure. The root element is `<robot>` which contains all other elements. Key components of a URDF file include:

- **Robot element**: The root element that defines the robot name and contains all other elements
- **Link elements**: Represent rigid parts of the robot (e.g., chassis, arms, links)
- **Joint elements**: Define connections between links with specific kinematic properties
- **Material elements**: Define visual appearance properties
- **Gazebo elements**: Define simulation-specific properties

### Links in URDF

Links represent the rigid bodies of a robot. Each link can have multiple properties:

- **Inertial properties**: Mass, center of mass, and inertia tensor
- **Visual properties**: How the link appears in visualization
- **Collision properties**: How the link interacts in collision detection
- **Optional properties**: Name, origin transformations

A link element typically contains:
- `<inertial>`: Mass and inertia properties for dynamics simulation
- `<visual>`: Geometry and material for visualization
- `<collision>`: Geometry for collision detection

### Joints in URDF

Joints define the connections between links and specify how they can move relative to each other. Joint types include:

- **Revolute**: Rotational joint with limited range
- **Continuous**: Rotational joint without limits
- **Prismatic**: Linear sliding joint with limited range
- **Fixed**: No movement (used to attach links rigidly)
- **Floating**: 6-DOF movement (rarely used)
- **Planar**: Movement in a plane

Each joint defines:
- Parent and child links
- Joint type
- Origin (position and orientation relative to parent)
- Axis of rotation/translation
- Limits (for revolute and prismatic joints)
- Dynamics properties (damping, friction)

### Coordinate Systems and Transformations

URDF uses a right-handed coordinate system where:
- X-axis points forward
- Y-axis points left
- Z-axis points up

Transformations between coordinate frames are represented using origin elements that specify position (x, y, z) and orientation (roll, pitch, yaw) relative to the parent frame.

### Visual and Collision Properties

URDF distinguishes between visual and collision representations:
- **Visual elements**: Define how a link appears in visualization tools
- **Collision elements**: Define geometry for collision detection and physics simulation

These can be the same for simple shapes but often differ for computational efficiency (simpler collision geometry than visual geometry).

### Geometric Shapes in URDF

URDF supports various geometric shapes:
- **Box**: Rectangular parallelepiped
- **Cylinder**: Cylindrical shape
- **Sphere**: Spherical shape
- **Mesh**: Complex shapes using external mesh files (STL, DAE, OBJ)

### Humanoid-Specific Considerations

Humanoid robots present unique challenges in URDF modeling:

1. **Complex Kinematic Structure**: Humanoids typically have multiple limbs with redundant degrees of freedom, requiring careful kinematic chain design.

2. **Symmetry**: Humanoid robots often have symmetrical limbs (left/right arms, legs), which can be modeled efficiently using parameterized designs.

3. **Balance Requirements**: Humanoid models must consider center of mass and balance, particularly important for bipedal locomotion.

4. **Anthropomorphic Dimensions**: Joint limits and link dimensions should reflect human-like proportions and capabilities.

5. **Degrees of Freedom**: Humanoids require many DOFs to replicate human-like movement, leading to complex URDF files.

### URDF Best Practices

- Use consistent naming conventions for links and joints
- Organize complex robots hierarchically
- Use appropriate level of detail for visual vs. collision geometry
- Include proper inertial properties for dynamic simulation
- Validate URDF files using tools like check_urdf
- Use Xacro (XML Macros) for parameterized and reusable robot descriptions

### URDF and ROS Integration

URDF integrates with ROS through:
- robot_state_publisher: Publishes transforms based on joint states
- joint_state_publisher: Publishes joint states for visualization
- TF (Transform) tree: Creates the coordinate frame hierarchy
- Simulation environments: Gazebo, RViz, and other tools

### Validation and Debugging

URDF files should be validated for:
- XML syntax correctness
- Kinematic chain completeness
- Proper parent-child relationships
- Consistent coordinate systems
- Valid joint limits and dynamics properties

Common validation tools include check_urdf, xacro, and visualization in RViz.

### URDF Extensions and Related Formats

While URDF is the standard, related formats include:
- **SDF (Simulation Description Format)**: Used by Gazebo
- **Xacro**: XML macro language that extends URDF with parameterization
- **MJCF (MuJoCo XML)**: Used by the MuJoCo physics engine

This foundational understanding of URDF is essential for creating accurate robot models, particularly for complex humanoid robots that require precise kinematic and dynamic representations.