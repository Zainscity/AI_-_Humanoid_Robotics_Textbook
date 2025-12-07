# Chapter 6: Modeling a Humanoid with URDF

Before we can simulate our robot, we need a detailed digital model of it. In ROS, the standard format for describing a robot's structure is the **Unified Robot Description Format (URDF)**. This chapter covers the essentials of creating a URDF file, from basic links and joints to more advanced concepts needed for accurate simulation.

## What is URDF?

URDF is an XML-based format that describes a robot's physical structure as a tree of links and joints. It defines the robot's kinematics (how its parts move relative to each other) and dynamics (how it responds to forces).

- **Links**: These are the rigid body parts of the robot (e.g., torso, upper arm, foot).
- **Joints**: These connect the links and define the motion between them.

## Key URDF Elements

-   **`<robot>`**: The root element of any URDF file.
-   **`<link>`**: Defines a rigid body part. It contains elements for its visual appearance, collision geometry, and inertial properties.
-   **`<joint>`**: Defines the kinematic relationship between two links (`parent` and `child`). Its `type` attribute determines the allowed motion (e.g., `revolute`, `prismatic`, `continuous`, `fixed`).

### Visual vs. Collision Geometry

A link has separate elements for its `<visual>` and `<collision>` geometry.
-   **`<visual>`**: This defines how the link looks in visualization tools like RViz. It can be a simple shape (box, cylinder, sphere) or a detailed 3D mesh (e.g., `.dae` or `.stl` file). Visual geometry can be complex and colorful.
-   **`<collision>`**: This defines the shape of the link for physics simulation. For performance, collision geometry is often a simplified version of the visual model (e.g., a collection of simple shapes that approximate the mesh).

### Inertial Properties

For a realistic physics simulation, you must define the inertial properties of each link using the `<inertial>` tag.
-   **`<mass>`**: The mass of the link in kilograms.
-   **`<inertia>`**: The 3x3 rotational inertia matrix, which describes the link's resistance to rotational motion about its center of mass.
-   **`<origin>`**: The pose of the center of mass relative to the link's origin.

```xml
<inertial>
  <mass value="1.5" />
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" />
</inertial>
```
Accurately defining these properties is critical for stable and predictable simulation behavior.

## Simplifying with XACRO

Writing a URDF for a complex humanoid robot by hand would be incredibly tedious and repetitive. **XACRO** (XML Macros) is a macro language that helps simplify this process. With XACRO, you can:
-   Define constants for commonly used values (`<xacro:property name="wheel_radius" value="0.1"/>`).
-   Create macros for reusable components (e.g., a macro for a complete leg, which can be instantiated twice).
-   Perform simple mathematical calculations.

XACRO files are processed by a tool that generates a final URDF file. This makes your robot description more modular, readable, and easier to maintain.

Here is a snippet using XACRO properties and a macro:

```xml
<xacro:property name="leg_length" value="0.8" />

<xacro:macro name="create_leg" params="prefix parent">
  <joint name="${prefix}_hip_joint" type="revolute">
    <parent link="${parent}"/>
    <child link="${prefix}_upper_leg"/>
    ...
  </joint>
  ...
</xacro:macro>

<!-- Instantiate the macro for the left and right legs -->
<xacro:create_leg prefix="left" parent="torso"/>
<xacro:create_leg prefix="right" parent="torso"/>
```

## Gazebo Extensions for Simulation

While URDF describes the robot's model, it doesn't cover everything needed for simulation. For example, it doesn't specify colors, materials, friction, or controller plugins. These are added using Gazebo-specific extensions within the URDF, enclosed in a `<gazebo>` tag.

Key Gazebo extensions include:
-   **`<material>`**: Sets the color and texture of a link (e.g., `<material>Gazebo/Orange</material>`).
-   **`<mu1>` and `<mu2>`**: Define the friction coefficients for a collision surface.
-   **`<sensor>`**: Adds a sensor plugin to a link, such as a camera, IMU, or contact sensor.
-   **`<plugin>`**: Loads a Gazebo plugin to control the robot, such as a differential drive controller or a PID controller for a joint.

```xml
<gazebo reference="wheel_link">
  <material>Gazebo/Black</material>
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
</gazebo>

<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/</robotNamespace>
  </plugin>
</gazebo>
```
These tags are ignored by tools that don't understand them (like RViz), but they are essential for configuring how your robot behaves in Gazebo.

## Sources & References

- [ROS Wiki - URDF](http://wiki.ros.org/urdf)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [ROS Wiki - XACRO](http://wiki.ros.org/xacro)
- [Gazebo Simulation - URDF](http://gazebosim.org/tutorials?tut=ros_urdf)