# PX4 Multi-Drone Simulation Project

This project is designed to simulate and control multiple drones in a PX4-based simulation environment. The system allows for the integration of various drone types, trajectory planning using G-codes, and interaction via ROS2 for advanced multi-agent robotics systems.

![Main GIF](path/to/your/main-gif.gif)
*Описание основной GIF*
---

## Table of Contents
- [PX4 Multi-Drone Simulation Project](#px4-multi-drone-simulation-project)
  - [*Описание основной GIF*](#описание-основной-gif)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
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