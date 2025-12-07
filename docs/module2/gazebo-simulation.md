# Chapter 7: Simulating the World with Gazebo

With a URDF model in hand, we can bring our robot to life in a simulated environment. **Gazebo** is a powerful 3D robotics simulator that allows you to test your robot's design and control algorithms in a realistic virtual world before deploying them on a physical robot.

## Why Gazebo?

Gazebo is a cornerstone of the ROS ecosystem for several reasons:
-   **Realistic Physics**: It uses high-performance physics engines (like ODE, Bullet, or DART) to simulate gravity, friction, contact forces, and fluid dynamics.
-   **High-Fidelity Sensors**: You can add a wide variety of sensors to your robot, including cameras, LiDAR, IMUs, GPS, and contact sensors. Gazebo generates realistic sensor data, including noise, which is crucial for testing perception algorithms.
-   **Seamless ROS Integration**: Gazebo is tightly integrated with ROS 2. Through the `gazebo_ros` package, you can spawn URDF models, control joints, and publish/subscribe to topics, services, and actions, just as you would on a real robot.
-   **Extensible Architecture**: Gazebo has a powerful plugin architecture that allows you to customize and extend the simulator's functionality to match your specific needs.

## The Gazebo World File

A Gazebo simulation is defined by a **world file** (with a `.world` extension). This is an XML file that describes everything in the environment: the lighting, the physics properties, and all the models (robots, tables, walls, etc.).

Here's a snippet from a simple world file:

```xml
<sdf version="1.7">
  <world name="default">
    <!-- Set the lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- Add a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <!-- Set the physics engine properties -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <!-- Include a robot model -->
    <include>
      <name>my_robot</name>
      <uri>model://my_robot_model</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>
  </world>
</sdf>
```

You can launch Gazebo with a specific world file from a launch file.

## Gazebo Plugins

Plugins are shared libraries that allow you to extend Gazebo's functionality. There are several types:
-   **World Plugins**: Affect the entire world (e.g., a plugin to modify wind or gravity).
-   **Model Plugins**: Are attached to a specific model and are used to control its behavior. The most common use case is to implement a robot's controller.
-   **Sensor Plugins**: Are attached to a sensor and generate its data (e.g., a camera plugin that publishes images to a ROS topic).
-   **System Plugins**: Are loaded at startup and can access all elements of the simulation.

## Controlling the Robot with `ros2_control`

The standard way to control a robot in Gazebo is with the `ros2_control` framework. This is a set of packages that provides a generic interface for controllers. You define your robot's joints and actuators in your URDF, and then `ros2_control` loads the appropriate controllers (e.g., a `joint_trajectory_controller` for an arm or a `diff_drive_controller` for a mobile base).

To use it with Gazebo, you typically add the `gazebo_ros2_control` plugin to your URDF:

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <robot_param>robot_description</robot_param>
    <robot_param_node>robot_state_publisher</robot_param_node>
    <parameters>path/to/your/controllers.yaml</parameters>
  </plugin>
</gazebo>
```
This plugin reads your URDF and the controller configuration, and it sets up the necessary ROS 2 interfaces to send commands and receive feedback.

## Spawning and Managing Models

While you can include models directly in your world file, you often need to add or remove them dynamically. The `gazebo_ros` package provides services for this:
-   `/spawn_entity`: A service to spawn a new model into the simulation from a URDF or SDF file.
-   `/delete_entity`: A service to remove a model from the simulation.

You can call these services from a launch file (using `spawn_entity.py`) or from your own ROS 2 nodes.

Here is an updated launch file example showing how to load a world and spawn a robot from a URDF stored in a parameter:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Assume the robot URDF is loaded into a 'robot_description' parameter
    robot_description = LaunchConfiguration('robot_description')

    return LaunchDescription([
        # Launch Gazebo with a specific world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'path/to/my_world.world',
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Spawn the robot from the robot_description parameter
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_humanoid'],
            output='screen'
        ),
        # Publish the robot's state to TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
    ])
```

## Sources & References

- [Gazebo Documentation](http://gazebosim.org/docs)
- [ROS 2 Gazebo Tutorials](https://navigation.ros.org/setup_guides/simulator/setup_gazebo.html)
- [ros2_control Documentation](https://control.ros.org/master/index.html)
- [gazebo_ros_pkgs Repository](https://github.com/ros-simulation/gazebo_ros_pkgs)