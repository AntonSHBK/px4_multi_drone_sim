# PX4 Multi-Drone Simulation Project

This project is designed to simulate and control multiple drones in a PX4-based simulation environment. The system allows for the integration of various drone types, trajectory planning using G-codes, and interaction via ROS2 for advanced multi-agent robotics systems.

---

## Table of Contents
- [PX4 Multi-Drone Simulation Project](#px4-multi-drone-simulation-project)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Directory Structure](#directory-structure)
  - [Features](#features)
  - [Getting Started](#getting-started)
    - [Prerequisites](#prerequisites)
    - [Building the Environment](#building-the-environment)
  - [Usage](#usage)
    - [Fast Commands](#fast-commands)
    - [Launch Files](#launch-files)
  - [G-Code System](#g-code-system)
  - [Contributing](#contributing)
  - [License](#license)

---

## Overview
This project provides a modular and extensible framework for simulating drones in a PX4-based environment. Key components include:
- **Drone controllers** for managing the logic and behavior of drones.
- **G-code implementation** for defining drone trajectories and actions.
- **Dockerized environment** for consistent setup and deployment.
- **Multi-drone message protocols** for inter-drone communication and state management.

---

## Directory Structure

```plaintext
.
├── docker
│   ├── docker-compose.yml   # Docker-compose configuration for the environment.
│   └── Dockerfile           # Dockerfile for building the simulation container.
├── docs
│   ├── article              # Articles and guides related to the project.
│   ├── description          # Detailed description of the system and components.
│   ├── main.md              # Main documentation entry.
│   └── media                # Media assets for documentation.
├── fast_commands.md          # Quick reference for available commands.
├── LICENSE                   # License file.
├── multi_drone
│   ├── CMakeLists.txt        # Build configuration for ROS2 package.
│   ├── config                # Configuration files for drones and simulation.
│   │   └── test_params.yaml  # Example parameter file for tests.
│   ├── controllers           # Controllers for managing drone logic.
│   │   ├── base_controller.py # Base class for all drone controllers.
│   │   ├── base_data.py      # Data classes for managing drone states.
│   │   ├── r1_rover          # Implementation for R1 Rover controllers.
│   │   ├── rover_ackermann   # Implementation for Ackermann-style rovers.
│   │   └── x500              # X500 drone-specific controller implementations.
│   ├── g_code                # Implementation of G-code commands for drones.
│   │   ├── base_command.py   # Base class for G-code commands.
│   │   └── __init__.py       # Initialization for G-code commands.
│   ├── launch                # ROS2 launch files for running simulations.
│   │   └── test.launch.py    # Example launch file.
│   ├── package.xml           # ROS2 package metadata.
│   ├── scripts               # Auxiliary scripts for running servers and simulations.
│   ├── tests                 # Test cases and validation.
│   └── utils                 # Utility modules (geometry, transformations, etc.).
├── multi_drone_msg           # ROS2 message definitions.
│   ├── CMakeLists.txt        # Build configuration for the message package.
│   ├── msg                   # Message definitions.
│   │   └── RobotState.msg    # Example state message.
│   └── package.xml           # ROS2 package metadata for messages.
├── other                     # Other related modules and packages.
│   └── multi_dron_interface  # Example interface for multi-drone systems.
├── README.md                 # This readme file.
```

---

## Features

- **G-Code System**: 
  - Supports commands like linear motion, circular interpolation, takeoff, landing, etc.
  - Easily extendable for custom drone actions.
- **Multi-Drone Simulation**:
  - Simulate and control multiple drones simultaneously.
  - Real-time state monitoring and command execution.
- **ROS2 Integration**:
  - Full compatibility with ROS2 for node-based drone control.
  - Custom message types for inter-drone communication.
- **Docker Support**:
  - Simplifies setup and deployment of the simulation environment.
- **Extensible Controllers**:
  - Base classes for creating custom drone types (e.g., X500, R1 Rover).

---

## Getting Started

### Prerequisites
- Docker and Docker Compose
- ROS2 Humble or later
- PX4 Autopilot (compatible version)

### Building the Environment
1. Clone the repository:
   ```bash
   git clone https://github.com/AntonSHBK/px4_multi_drone_sim.git
   cd px4_multi_drone_sim
   ```
2. Build the Docker container:
   ```bash
   docker-compose up --build
   ```
3. Launch the simulation:
   ```bash
   ros2 launch multi_drone test.launch.py
   ```

---

## Usage

### Fast Commands
Refer to `fast_commands.md` for a list of quick commands and their descriptions.

### Launch Files
- Use `test.launch.py` to start a sample simulation.
- Modify configuration files in `multi_drone/config` for custom setups.

---

## G-Code System

The G-code system allows you to define drone actions and trajectories using a command-like syntax. Examples:
- **G0**: Stop
- **G20**: Move to a specific point
- **G21**: Linear motion between points
- **G22**: Circular interpolation

G-code commands are implemented in `multi_drone/g_code`.

---

## Contributing
We welcome contributions! Feel free to open issues or submit pull requests.

---

## License
<!-- This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details. -->