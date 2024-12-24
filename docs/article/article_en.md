# **Article: Group Control of Unmanned Aerial Vehicles (UAVs)**

---

## **Introduction**

In recent years, unmanned aerial vehicle (UAV) technology has rapidly developed, covering a wide range of fields from agriculture to search and rescue operations. The use of drone swarms opens up new opportunities, such as collective cargo delivery, monitoring vast areas, and performing complex tasks in coordination. However, effective management of a drone swarm requires comprehensive solutions, including high-level planning, stable communication, and automation of control.

The aim of this work is to present a group control system for UAVs using the ROS 2 framework and PX4 autopilot, providing flexibility, scalability, and the ability to execute complex scenarios.

---

## **Overview of Related Works**

Numerous studies in the field of UAV control focus on solving issues related to coordination and planning. Let us review some achievements in this area:

1. **Swarm Control:**
   - The use of swarm intelligence algorithms, such as PSO (Particle Swarm Optimization), allows a group of drones to coordinate their actions to achieve a common goal.

2. **Decentralized Control:**
   - In the works of [Guerrero-Bonilla et al., 2020] and [Chung et al., 2018], a decentralized approach is explored, allowing drones to make decisions based on local information, reducing dependency on a central controller.

3. **Simulation and Testing:**
   - Systems such as Gazebo and AirSim provide powerful tools for modeling the physical environment, significantly accelerating the development of control algorithms.

However, many existing solutions face limitations in real-world integration or have narrow specializations. Our system aims to overcome these limitations.

---

## **System Description**

### **General Overview**

The proposed UAV control system is based on the following principles:
- **Modularity:** System components are isolated, simplifying their development, testing, and upgrades.
- **Flexibility:** The system can be adapted to various scenarios, including simulation and real drone operations.
- **Scalability:** The system supports operation with multiple drones without significant changes to the architecture.

---

### **Technology Stack**

1. **Operating System:** Ubuntu 22.04 LTS
2. **Framework:** ROS 2 Humble
3. **Simulation:** Gazebo Ignition
4. **Autopilot:** PX4 with MAVROS integration

---

### **System Components**

1. **Centralized Control:**
   - The central node collects data from all drones, analyzes the current situation, and sends commands to achieve a common goal.

2. **Drone Controllers:**
   - Each drone is equipped with its own controller responsible for executing commands, receiving telemetry, and managing modes.

3. **G-code Command System:**
   - High-level commands such as `G20_MoveToPoint` and `G21_LinearMove` define actions for the drones. These commands simplify the development of control scenarios.

4. **Offboard Mode:**
   - This mode is used for precise control of drones, including sending trajectories and waypoints.

---

### **Scenario Example**

The system supports complex scenarios such as coordinating a group of drones for area monitoring. For example:

1. **Initialization:**
   - Each drone is loaded into the Gazebo simulation and receives parameters from the configuration file.

2. **Task Assignment:**
   - The central node assigns tasks to drones using G-codes.

3. **Task Execution:**
   - Drones execute commands, moving along the trajectories sent via the G-code command system.

4. **Feedback:**
   - Each drone publishes its status data, allowing the central node to adapt subsequent actions.

---

### **System Achievements**

1. **Versatility:**
   - Support for both simulation and real-world drone operations.
2. **Ease of Integration:**
   - Simplicity of adding new commands and scenarios.
3. **Complex Trajectory Implementation:**
   - Ability to define spirals, circles, straight-line movements, and more.

---

## **Conclusion**

The presented UAV control system demonstrates a high degree of adaptability and scalability. It is suitable for tasks in various fields, including agriculture, monitoring, and rescue operations.

Future work focuses on implementing machine learning algorithms to improve drone coordination and increase the system's autonomy.