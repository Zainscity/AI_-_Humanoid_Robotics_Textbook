# Appendix A: Hardware Guide

This appendix provides a detailed guide to the hardware required to follow the hands-on portions of this book, both for the digital twin workstation and for building a physical robot.

## Digital Twin Workstation

A powerful workstation is essential for running the simulations, especially the GPU-intensive NVIDIA Isaac Sim.

-   **Operating System**: Ubuntu 22.04 LTS. This is the primary target for ROS 2 Humble and the NVIDIA Isaac SDK.
-   **CPU**: Intel Core i7 / AMD Ryzen 7 (8 cores or more recommended). A powerful CPU is needed for compiling code, running multiple ROS nodes, and general system responsiveness.
-   **RAM**: 32 GB DDR4 or more. Isaac Sim and other robotics development tools are memory-intensive. 64 GB is recommended for a smoother experience, especially when running large simulations.
-   **GPU**: NVIDIA RTX 3070 or higher. An RTX GPU is **required** for Isaac Sim's real-time ray tracing capabilities. The more VRAM your GPU has, the larger and more complex the scenes you can simulate.
-   **Storage**: 100 GB of free SSD storage. An SSD is critical for fast loading times for the OS, simulation environments, and large datasets.

## Physical Robot Components

For readers who wish to build a physical version of the humanoid robot, here is a list of recommended components.

### The "Brain": AI Edge Computer

-   **NVIDIA Jetson AGX Orin Developer Kit**: This is highly recommended as the onboard computer for the robot. It provides a powerful CPU and an integrated GPU that can run the entire perception and control stack, including the Isaac ROS packages, directly on the robot. This enables a high degree of autonomy.

### Sensors

-   **Depth Camera**:
    -   **Intel RealSense D435i or D455**: These are excellent choices that provide synchronized color, depth, and infrared streams, along with an integrated Inertial Measurement Unit (IMU). The IMU is critical for state estimation and balance control.
-   **Microphone Array**:
    -   **ReSpeaker 4-Mic Array**: A multi-microphone array is recommended over a single microphone. It allows for beamforming and noise cancellation, which significantly improves the performance of voice recognition with Whisper, especially in noisy environments.

### Power Systems

Powering a mobile robot is a non-trivial challenge. You will need:
-   **High-Discharge LiPo Batteries**: To provide the high current required by the motors.
-   **Battery Management System (BMS)**: To ensure safe charging and discharging of the batteries.
-   **Voltage Regulators (BECs)**: To provide stable 5V and 12V power to the computer and sensors from the main battery.

### Robot Platform Options

Building a full humanoid robot from scratch is a significant undertaking. The principles in this book can be adapted to various platforms.

-   **Unitree G1**: For those with a significant budget, the Unitree G1 is a highly capable humanoid that can serve as an excellent research platform.
-   **3D Printed Humanoid (e.g., InMoov, Poppy)**: For the ambitious maker, there are several open-source 3D printed humanoid projects. These are a great way to learn about the mechanics and electronics of robotics, but they require a significant time investment.
-   **Robot Arm on a Mobile Base**: A practical starting point is to mount a robot arm (like a WidowX or a UR5) on a mobile base (like a TurtleBot). This allows you to work on navigation and manipulation tasks without the immense complexity of bipedal locomotion.

### Networking and Connectivity

A reliable network is crucial.
-   **Wi-Fi Router**: A good quality Wi-Fi router is needed for communication between your workstation and the robot.
-   **Ethernet**: For stationary testing, a direct Ethernet connection can provide a more stable and lower-latency link.

## Budget Considerations (Estimates)

-   **Simulation Only**: The cost of the workstation (approx. $2000 - $4000 USD).
-   **Basic Physical Robot (Arm on Mobile Base)**: Workstation + mobile base + arm + sensors (approx. $5000 - 0,000+ USD).
-   **Full Humanoid**: The cost can vary dramatically, from a few thousand dollars for a 3D-printed version (if you have the printer and tools) to tens or hundreds of thousands of dollars for a commercial research platform.

## Lab Architecture: On-Prem vs. Cloud

-   **On-Prem**: Having a dedicated workstation and physical robot in your own lab provides the most flexibility and hands-on experience.
-   **Cloud**: If you do not have access to the recommended hardware, a significant portion of this book can be followed using a cloud-based GPU instance (e.g., on AWS, GCP, or Azure) to run the simulations. The book provides guidance on setting up such an environment.

## Sources & References

- [NVIDIA Jetson AGX Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
- [Intel RealSense D455](https://www.intelrealsense.com/depth-camera-d455/)
- [ReSpeaker 4-Mic Array for Raspberry Pi](https://www.seeedstudio.com/ReSpeaker-4-Mic-Array-for-Raspberry-Pi-p-3048.html)
- [Unitree Robotics](https://www.unitree.com/)
- [InMoov Humanoid Robot](http://inmoov.fr/)