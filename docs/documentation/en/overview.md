### Overview

<img src="../../media/logo.webp" alt="Project Logo" width="600"/> 

This project focuses on the development and implementation of an unmanned aerial vehicle (UAV) control system using ROS 2 and PX4 flight controllers. The system is designed to streamline the management of UAVs in a simulation environment while ensuring modularity, scalability, and precision in drone operations.

#### Key Features:
1. **Centralized and Modular Control:**
   - The system employs a centralized control mechanism for coordinating the group. The central unit collects real-time data from all drones, calculates actions to achieve group goals, and sends commands to the drones.

2. **State-Based Architecture:**
   - Drone states such as idle, arming, takeoff, flight planning, offboard control, and landing are implemented using the State design pattern. Each state is represented by a class, making transitions and logic modular and easy to manage.

3. **High-Level Command Execution:**
   - Inspired by G-code systems in CNC, the system uses simplified JSON-based commands for drone operations. Each command is encapsulated in a class that manages its execution, status checks, and completion criteria.

4. **Simulation and Testing:**
   - The Gazebo simulation environment is used to model and test real-world scenarios with multiple UAVs. Configurations for drones and environments are defined in YAML files for flexibility.

5. **Technology Stack:**
   - **OS:** Ubuntu 22.04 LTS
   - **Framework:** ROS 2 Humble
   - **Languages:** Python
   - **Simulator:** Gazebo Ignition
   - **Flight Controller:** PX4 with MAVROS integration

---

#### System Architecture:
1. **Components:**
   - Drone controllers manage individual UAV operations.
   - The group controller coordinates tasks among multiple UAVs.
   - Decision-making and navigation modules enable dynamic route planning and obstacle avoidance.

2. **Data Exchange:**
   - Communication is done through ROS 2 topics and services.
   - MAVROS bridges ROS and PX4, facilitating command and telemetry exchange.

3. **Integration with Physical UAVs:**
   - The system supports seamless transition between simulated and real UAVs, allowing the addition of new drones without significant reconfiguration.

---

#### Development Highlights:
- Modular code structure for ease of customization and extension.
- Dockerized environment for stable and repeatable deployment.
- Extensive documentation covering setup, usage, and integration scenarios.
- Real-world testing with multiple UAVs demonstrated high accuracy in task execution.

#### Applications:
- Agriculture, mapping, and surveying.
- Search and rescue operations.
- Autonomous inspection and delivery systems.

#### Future Work:
- Implementation of swarm intelligence algorithms for decentralized control.
- Improving real-world adaptability with enhanced obstacle detection and avoidance systems.

This project provides a robust platform for managing UAVs in both simulated and real-world conditions, opening new opportunities for research and applications in autonomous aerial systems.