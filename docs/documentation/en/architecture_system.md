### System Architecture

This document describes the architecture of a group control system for unmanned aerial vehicles (UAVs), emphasizing its modularity, scalability, and extensibility. The architecture combines centralized group control with distributed modules for controlling individual drones, ensuring reliability and adaptability.

---

#### **1. General Overview of the Architecture**

The system architecture is designed to control a group of UAVs using ROS 2 and PX4. The main components include:
- **Centralized Control Unit (CCU):** collects telemetry, computes actions, and sends commands to drones.
- **Drone Controllers:** manage the operation of each drone and execute received commands.
- **Communication System:** facilitates message exchange between components using ROS 2 topics and services.
- **Simulation Environment:** implemented on Gazebo for modeling real-world physical conditions.

The architecture supports modularity by isolating each function into separate packages and components, making it easier to add new features and modify the system.

---

#### **2. Key Components**

##### **2.1 Centralized Control Unit (CCU)**
The CCU performs the following tasks:
- Collecting telemetry data from all drones.
- Computing group actions to achieve a common goal.
- Sending commands to individual drones via ROS topics.

##### **2.2 Drone Controller**
Each drone is controlled by its individual controller, which is responsible for:
- Maintaining the current state (e.g., idle, takeoff, flight).
- Interpreting high-level commands and converting them into PX4-compatible instructions.
- Publishing telemetry data, such as position, speed, and orientation.

Controllers implement the **State design pattern**, where each state (e.g., takeoff, position hold) is represented by a class with methods for initialization, execution, and transition logic.

##### **2.3 Communication System**
The system uses ROS 2 topics and services for:
- **Telemetry:** publishing current drone status data.
- **Commands:** transmitting high-level instructions to drones.
- **Feedback:** reporting command execution status and completion.

##### **2.4 Simulation Environment**
- **Gazebo Ignition** models drones, the environment, and physical processes.
- A YAML configuration defines drone parameters and initial positions.
- The simulation supports testing scenarios of varying complexity, including drone coordination and obstacle avoidance.

---

#### **3. Workflow Logic**

##### **Step 1: Initialization**
- The CCU launches the simulation environment based on a predefined configuration.
- Drones are placed in the Gazebo environment, each with an instance of PX4.

##### **Step 2: Data Collection**
- Telemetry data (position, orientation, speed) is published to ROS topics.
- The CCU collects and analyzes telemetry to assess the group’s state.

##### **Step 3: Decision Making**
- The CCU computes actions based on goals, constraints, and environmental data.
- Actions are encoded in high-level JSON commands.

##### **Step 4: Command Execution**
- Commands are sent to the controllers of individual drones.
- Controllers convert high-level commands into PX4-compatible messages through MAVROS.

##### **Step 5: Feedback and Monitoring**
- Drone controllers report the status of command execution back to the CCU.
- RViz provides real-time visualization of drone status and trajectories.

---

#### **4. Modular Design**

##### **4.1 Packages and Modules**
- **Group Control Package:** manages group dynamics and distributes commands.
- **Individual Control Package:** handles states and telemetry for a single drone.
- **Command Processing Package:** interprets and processes JSON commands.
- **Navigation Module:** implements trajectory planning and obstacle avoidance.
- **Simulation Package:** manages the launch and setup of the Gazebo environment.

##### **4.2 Extensibility**
- To add a new drone, simply define parameters in the YAML file.
- New commands are implemented by inheriting the `BaseGCommand` class.
- Additional features, such as swarm intelligence algorithms, can be integrated as separate modules.

---

#### **5. Advantages**

1. **Modularity:**
   - Clear separation of group and individual drone control logic.
   - Easy to add or modify components without impacting the entire system.

2. **Scalability:**
   - Supports dynamic addition of new drones and controllers.
   - Adaptable to both simulated and real-world environments.

3. **Flexibility:**
   - High-level commands simplify control logic.
   - ROS 2 ensures reliable distributed communication.

4. **Real-time Monitoring:**
   - Telemetry and visualization tools, such as RViz, simplify debugging and analysis.

---

#### **6. Limitations**
- Dependency on MAVROS and PX4 may introduce slight delays under heavy loads.
- Simulation accuracy may vary due to hardware or software limitations.