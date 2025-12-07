# Chapter 8: High-Fidelity Simulation with Unity

While Gazebo is excellent for physics simulation, the **Unity** game engine offers a powerful alternative for creating high-fidelity, visually rich simulation environments. Unity is particularly strong in areas like realistic rendering, complex sensor simulation, and creating large, interactive worlds, making it an ideal choice for training and testing perception and HRI algorithms.

## Why Unity for Robotics?

Unity has become a leading platform for robotics simulation thanks to a set of specialized tools. The [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) provides open-source packages that enable seamless integration with ROS 2.

Key advantages include:
-   **Photorealistic Graphics**: Unity's High Definition Render Pipeline (HDRP) allows for stunningly realistic visuals, which is crucial for training and validating computer vision models on synthetic data.
-   **ROS 2 Integration**: The `ROS-TCP-Connector` package allows Unity to communicate with a ROS 2 network over TCP. The `Visualizations` package provides out-of-the-box ways to display ROS messages (like point clouds and trajectories) in the Unity scene.
-   **URDF Importer**: The `URDF-Importer` package can import a URDF file and automatically generate a corresponding robot model in Unity, including its visual meshes and joint hierarchy.
-   **Rich Asset Ecosystem**: The Unity Asset Store provides a vast library of 3D models, textures, and environments that you can use to rapidly build complex and diverse simulation worlds.

## The Unity Simulation Environment

A Unity project is structured around **Scenes**. A scene contains all the **GameObjects** that make up your environment. Each GameObject has a set of **Components** that define its behavior and appearance. For example, a robot link would be a GameObject with components for its mesh, material, and physics properties.

### Articulated Bodies

To create physically accurate robots, Unity provides the **`ArticulationBody`** component. An articulation is a tree-like structure of bodies connected by joints. This is a perfect match for the link-and-joint structure of a URDF.

When you import a URDF, Unity automatically creates a hierarchy of `ArticulationBody` components. Each component has properties to define the joint type (revolute, prismatic), limits, friction, and stiffness. This system is optimized for the kind of stability needed for robotics, which is often a challenge for traditional game physics engines.

## Controlling a Robot in Unity

You control a robot in Unity by writing C# scripts that interact with the `ArticulationBody` components. These scripts can subscribe to ROS 2 topics to receive commands.

For example, to control a joint, you would subscribe to a topic publishing `Float64` messages and then set the target position of the corresponding `ArticulationBody`.

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class JointController : MonoBehaviour
{
    private ArticulationBody articulation;

    void Start()
    {
        articulation = GetComponent<ArticulationBody>();
        ROSConnection.GetOrCreateInstance().Subscribe<Float64Msg>("my_joint_commands", SetJointTarget);
    }

    void SetJointTarget(Float64Msg message)
    {
        var drive = articulation.xDrive;
        drive.target = (float)message.data;
        articulation.xDrive = drive;
    }
}
```

This C# script, when attached to a GameObject with an `ArticulationBody`, will listen for commands on the `my_joint_commands` topic and drive the joint to the received target position.

## Simulating Sensors in Unity

Unity is an excellent platform for simulating sensors.
-   **Cameras**: You can add a `Camera` component to your robot and configure it to match the properties of a real-world camera (focal length, resolution, etc.). You can then write a script to read the rendered images, convert them to a ROS 2 message format, and publish them.
-   **LiDAR**: Simulating a LiDAR scanner can be done by performing many raycasts out from a central point and recording the hit distances. This data can then be assembled into a `LaserScan` or `PointCloud2` message.
-   **IMUs**: You can read the `ArticulationBody`'s velocity and angular velocity to simulate an IMU.

The Unity Robotics Hub provides examples and packages that can help you get started with building your own sensor models.

In this book, we will primarily use Gazebo for its deep integration with `ros2_control`, but we will explore scenarios where Unity's rendering capabilities provide a distinct advantage, particularly for developing and testing perception systems.

## Sources & References

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [Unity's ArticulationBody Documentation](https://docs.unity3d.com/Manual/class-ArticulationBody.html)