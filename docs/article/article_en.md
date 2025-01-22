# **Article: Group Control of Unmanned Aerial Vehicles (UAVs)**

- [**Article: Group Control of Unmanned Aerial Vehicles (UAVs)**](#article-group-control-of-unmanned-aerial-vehicles-uavs)
    - [Abstract](#abstract)
    - [1. **Introduction**](#1-introduction)
    - [2. **Review of Existing Solutions**](#2-review-of-existing-solutions)
    - [3. **System Architecture**](#3-system-architecture)
    - [4. **High-Level Controller Implementation**](#4-high-level-controller-implementation)
    - [5. **Experimentation**](#5-experimentation)
    - [6. **Results Analysis**](#6-results-analysis)
    - [7. **Conclusion**](#7-conclusion)
    - [**References**](#references)

### Abstract

This paper presents the development and testing of a multi-drone management system implemented using the ROS 2 framework, PX4 software, and the Gazebo simulation environment. The focus is on the implementation of a high-level controller that uses G-commands to standardize drone management. The conducted experiments included testing the accuracy of command execution, drone coordination, and trajectory analysis.

The results demonstrated high accuracy in performing tasks such as takeoff, movement, and stabilization. However, shortcomings were identified, including initial positioning errors and command execution asynchrony. The paper provides recommendations for system improvements, including the development of calibration and synchronization mechanisms, as well as the integration of additional functional modules such as odometry and autonomous navigation. This work lays the foundation for further development of UAV management systems capable of operating effectively in both simulated and real-world environments.

### 1. **Introduction**

With the advancement of technology, unmanned aerial vehicles (UAVs) are becoming increasingly popular across various industries, including logistics, agriculture, construction, security, and scientific research. Their flexibility and ability to efficiently perform tasks in challenging conditions have led to a significant increase in the number of deployed devices. However, the growing scale of their use necessitates the development of more efficient control methods, especially in scenarios where a single operator must coordinate multiple drones simultaneously.

Modern approaches to UAV management rely on advanced hardware and software platforms, such as PX4 Autopilot and the ROS 2 framework, which have become de facto standards in the field of robotics. PX4 is a versatile open-source flight control platform that supports multiple types of devices and offers a wide range of features, including high-precision navigation, task automation, and integration with various sensors [1]. A comparable platform to PX4 is ArduPilot, which provides extensive functionality for UAV management [2]. On the other hand, ROS 2 (Robot Operating System) offers a flexible tool for developing distributed control systems, which is particularly important for coordinating UAV groups and their interactions in dynamic environments [3].

One promising approach is the adoption of the G-command concept, inspired by numerical control (NC). The use of such commands standardizes the interaction between various system components, serving as a sort of interface or API for data exchange and control commands. G-commands define high-level task execution logic, ensuring ease of integration for new functional capabilities and modules [4]. This approach simplifies the development of complex control systems and enhances their compatibility with diverse hardware and software.

The goal of this work is to develop and test a high-level controller for UAV management using G-commands, operating on the PX4 and ROS 2 platforms. The controller is expected to enable the transmission and execution of simplified commands, such as point-to-point movement, linear movement, circular orbiting, and subsequent real-time drone status monitoring.

To validate the proposed approach, a simulation environment was created to evaluate the efficiency and reliability of the developed system. The paper presents the structure and operational principles of the proposed controller.

### 2. **Review of Existing Solutions**

Modern solutions for UAV management cover a wide range of tasks, including autonomous navigation, group action coordination, and integration of various control systems. One of the most widely used platforms is PX4 Autopilot, which is employed for drone flight control in both real and simulated environments. This platform supports various communication protocols, such as MAVLink, and facilitates integration with high-level frameworks.

The ROS 2 framework is actively used for developing distributed UAV control systems. Its use enables the creation of modular systems where data exchange between components is carried out through topics and services. ROS 2 is applied to tasks such as navigation, obstacle avoidance, trajectory planning, and drone interaction within swarm systems [3].

A key challenge is the coordination of group actions for multiple drones. Several studies explore swarm intelligence methods, enabling drones to operate as a cohesive unit. For instance, algorithms based on reinforcement learning principles have been successfully applied to coordinate group actions in complex dynamic conditions [5]. Other approaches, such as the CC-MPSO search method proposed by Chen, involve cooperative co-evolution and motion encoding, considering target motion models based on the Markov process under target motion uncertainty [6].

Autonomous control systems are also being developed using the concept of high-level commands, such as G-commands. These commands standardize interaction between system components and facilitate the integration of new functional modules [7]. The use of G-commands simplifies the development of complex control systems, ensuring their scalability and compatibility.

### 3. **System Architecture**

The UAV management system is built on the interaction of three key components: the PX4 autopilot platform, the ROS 2 framework, and the high-level controller. PX4 SITL provides flight simulation and low-level command processing, ROS 2 acts as a mediator for data and control transmission, and the controller processes high-level commands and coordinates drone actions.

Communication between PX4 and ROS 2 is implemented through MAVROS, which uses the MAVLink protocol to transmit commands and data. For message routing, the Micro XRCE-DDS Agent is utilized, enabling efficient information exchange in a distributed system.

The architecture and source code of the system are available on GitHub: PX4 Multi-Drone Simulation Project [8].

**System Modules:**

1. **Flight Control Module:** Includes PX4 SITL and MAVROS. It processes low-level commands and simulates flights. System parameters are configured through YAML files, providing flexibility and adaptability to various scenarios.

2. **Environment Generation Module:** Creates the simulation environment and drone models. The "Wall" standard world is used with options to add sensors (cameras, lidars) and modify environment parameters. Objects are described in SDF format.

3. **High-Level Controller:** Manages drone states (idle, arming, takeoff, position hold, landing) and supports ENU and NED coordinate systems for position and orientation transformations. The controller interacts with MAVROS to send commands to PX4.

4. **G-Command Processing Module:** Accepts and interprets commands in JSON format transmitted through named data channels. The commands define high-level logic, such as movement, takeoff, position hold, and landing. The command queue is processed sequentially.

5. **Monitoring and Visualization Module:** Uses RViz to display drone trajectories and states in real time. This module is employed to analyze command execution and diagnose the system.

6. **Drone Group Management Module:** Implements multi-drone coordination using swarm intelligence algorithms. It supports tasks such as obstacle avoidance, formation movement, and achieving common goals.

Data and command transmission in the system are implemented via named data channels, enabling organized communication between modules. Examples of channels:
- **G-Code Command Channels:** Used to transmit JSON commands to the high-level controller.
- **Monitoring Channels:** Contain data on drone states, current and target positions.
- **Command Processing Queues:** Ensure sequential task execution, completing the current command before starting the next.

The use of ROS 2 services and named data channels simplifies the integration of new functional modules and ensures system scalability. This architecture allows effective management of both individual drones and their groups, ensuring stability and precise command execution.

### 4. **High-Level Controller Implementation**

High-level commands implemented in G-code format are designed to control drone actions. They are structured as JSON objects transmitted via named ROS 2 data channels. Each command consists of a unique identifier (`command`), parameters (`parameters`), and an execution counter (`counter`). For instance, a command for linear drone movement includes parameters for the start and end coordinates, orientation angle, speed, and the coordinate system (ENU/NED, local/global).

An example of a G21 command for linear movement transmitted via ROS 2 is shown below. The command is published to the named channel `/id_1_x500/in/command_json` in JSON format [8]:

```bash
ros2 topic pub --once /id_1_x500/in/command_json std_msgs/msg/String "{data: '{
    \"name\": \"G21\",
    \"counter\": 2,
    \"start_point\": [0.0, 0.0, 0.0],
    \"end_point\": [5.0, 10.0, -10.0],
    \"velocity\": 1.5,
    \"yaw\": 1.57,
    \"coordinate_system\": \"global_ENU\"
}'}"
```

**Command Description:**
- **name:** Command identifier (G21) indicating a linear movement task.
- **counter:** Unique command counter for execution tracking.
- **start_point:** Starting point of the movement in the global ENU coordinate system.
- **end_point:** Endpoint of the movement in the global ENU coordinate system.
- **velocity:** Movement speed (m/s).
- **yaw:** Orientation angle (rad).
- **coordinate_system:** The coordinate system in which the movement is defined (`global_ENU` in this case).

This command specifies the drone's movement from the starting point `[0.0, 0.0, 0.0]` to the endpoint `[5.0, 10.0, -10.0]` with the given speed, orientation angle, and using the global ENU coordinate system.

**G-Command Processing Workflow:**
1. **Initialization:** The command is transmitted via a named channel, received by the controller, and deserialized.
2. **Execution:** The controller validates the command and passes the corresponding parameters to the PX4 autopilot through MAVROS. Command execution is monitored for progress.
3. **Completion:** Once executed, the command is marked as completed, and control is passed to the next command in the queue.

Commands are processed sequentially, simplifying management and ensuring control over each action's execution. A complete list of commands is available in the source code.

**State-Based Management:**
The controller uses a state design pattern to manage drone operation modes. Each mode (e.g., idle, takeoff, position hold, landing) is represented by a separate class with methods for initialization, execution, and completion. This ensures modularity and flexibility, simplifying the addition of new modes.

Supported States:
- **Idle:** Waiting state. The initial state after system startup.
- **Arming:** Drone arming. Prepares for flight.
- **Takeoff:** Ascends to the specified altitude.
- **Loiter:** Holds the current position.
- **Offboard:** External control mode for precise trajectory execution.
- **Landing:** Automatic landing.
- **Disarm:** Ends operation and transitions the drone to a safe state.

Transition logic between states depends on current system parameters. For instance, successful arming transitions the drone to takeoff, and takeoff completion transitions it to position hold.

The controller is implemented in Python using ROS 2 functionality for command processing, state management, and MAVROS interaction. The advantages of this approach include:
- **Ease of G-Command Handling:** The JSON format provides standardized interaction between modules.
- **Development Flexibility:** Python facilitates easy controller functionality expansion.
- **ROS 2 Integration:** Using ROS 2 services and data channels simplifies information exchange between modules.

Periodic command transmission at a high frequency (e.g., 0.1 seconds) ensures smooth control and avoids triggering emergency modes. This approach guarantees reliable command execution, even in complex flight scenarios.

```markdown
### 5. **Experimentation**

The experiment utilized modern tools and software to facilitate control, simulation, and data analysis. The system was based on the **ROS 2 Humble** framework, which provides a modular architecture and data exchange via named channels. The **Gazebo** simulation environment was used for realistic physical modeling and integration with ROS 2. Drone management was carried out using **PX4 Autopilot version 1.14**, connected to ROS through MAVROS.

The control logic and data processing were implemented in **Python 3.8** due to its simplicity and rich ecosystem of libraries. **RViz**, integrated with ROS, was employed for trajectory visualization and drone state monitoring. The **Docker** containerization platform was used to isolate the development environment and simplify system deployment. The entire system was developed and tested on the **Ubuntu 22.04 LTS** operating system.

The experiment aimed to test the coordination and interaction of a group of unmanned aerial vehicles (UAVs) in a shared environment. The primary objectives included testing the multi-drone control system, evaluating sequential command execution, and analyzing stability during simultaneous control of multiple drones. Control was conducted in the standard *default* simulation environment.

The drones were positioned at their starting coordinates at an altitude of 0 meters. To create a safe maneuvering space, the first step in each program was a takeoff command, moving the drones to an initial altitude of 5 meters.

Unique commands were sent to each drone via named ROS 2 data channels:
- **Drone 1 (id_1):** Performed forward movement of 5 meters with an orientation angle of 1.57 radians (90°) and a speed of 2.0 m/s in the local ENU coordinate system.  
  _Figure – Visualization of the first drone executing the command._

- **Drone 2 (id_2):** Executed linear movement from point (-5, 0, 5) to point (-5, -5, 5) at a constant speed of 1.5 m/s.  
  _Figure – Visualization of the second drone executing the command._

- **Drone 3 (id_3):** Performed an arc motion with a radius of 3 meters between points (0, 0, 5) and (5, 0, 5) in a counterclockwise direction.  
  _Figure – Visualization of the third drone executing the command._

### 6. **Results Analysis**

During the experiment, several shortcomings were identified, which should be addressed for future system improvements. Significant discrepancies were observed in the initial placement of the drone group (three units) at the starting coordinates. While sensor data indicated the drones were positioned as expected (_Figure, right_), visual inspection in the simulation environment showed a mismatch in actual positions (_Figure, left_). This discrepancy became particularly noticeable after executing takeoff commands, as confirmed by visualization results.

The likely cause of the discrepancy between reported and actual positions is related to the PX4 autopilot model's behavior in the simulation environment. These deviations might result from command processing delays, inaccuracies in the system's mathematical model, or data transmission errors through the simulation platform.

Analyzing the command execution for each drone individually demonstrated correct processing and adequate control. For example, commands to move to specified coordinates were executed visually correctly, as confirmed by trajectory visualization in *Gazebo* and *RViz*.

Despite the identified initial configuration issues, the system exhibited stability and precision in executing the given commands. This confirms its readiness for further testing and potential use in more complex scenarios. However, future developments should focus on improving initial placement accuracy and minimizing simulation-related errors.

### 7. **Conclusion**

A multi-drone management system was developed and tested using the ROS 2 framework, PX4 flight control software, and the Gazebo simulation environment. The primary focus was on basic control algorithms, drone coordination, and system testing in single and group scenarios.

The experiments demonstrated that the system could correctly execute commands, including takeoff, movement, and hovering, while showing resilience during various tasks. However, initial positioning errors and temporary asynchrony during group command execution were identified, requiring further refinement.

Future system development will focus on addressing these shortcomings, improving command synchronization mechanisms, creating a graphical user interface, and introducing odometry and autonomous navigation modules. These improvements will enhance the system's accuracy, reliability, and functionality, making it more effective for real-world applications.

### **References**

1. Meier, L., Honegger, D., Pollefeys, M. PX4: A node-based multithreaded open-source robotics framework for deeply embedded platforms // Proceedings - IEEE International Conference on Robotics and Automation. 2015. Vol. 2015. P. 6235-6240. DOI: 10.1109/ICRA.2015.7140074.

2. ArduPilot Development Team. ArduPilot Documentation [Online resource]. URL: https://ardupilot.org/ (accessed: 24.01.2025).

3. Macenski, S., Foote, T., Gerkey, B., Lalancette, C., Woodall, W. Robot Operating System 2: Design, Architecture, and Uses In The Wild. 2022. DOI: 10.48550/arXiv.2211.07752.

4. Abolarin, V. Generating G and M Code for Lathe Machine Using Python // 2024. Sep.

5. A Multi-Drone System Proof of Concept for Forestry Applications // Drones. 2025. Vol. 9. P. 80. DOI: 10.3390/drones9020080.

6. Chen, W., Fu, B., Wu, Z., Hao, L., Yang, G. Research on Dynamic Target Search for Multi-UAV Based on Cooperative Coevolution Motion-Encoded Particle Swarm Optimization // Applied Sciences. 2024. Vol. 14. P. 1326. DOI: 10.3390/app14041326.

7. Luan, N., Chen, J., Yan, Y. An Extended G-Code to be used on Networked Industrial Robots // International Journal of Robotics & Automation. 2004. Vol. 19. DOI: 10.2316/Journal.206.2004.2.206-2725.

8. Pisarenko, A. PX4 Multi-Drone Simulation Project [Online resource]. URL: https://github.com/AntonSHBK/px4_multi_drone_sim (accessed: 24.12.2024).
