# PX4 Multi-Drone Simulation Project

This project is designed to simulate and control multiple drones in a PX4-based simulation environment. The system allows for the integration of various drone types, trajectory planning using G-codes, and interaction via ROS 2 for advanced multi-agent robotics systems.

This work is part of my dissertation research. In the future, detailed textual descriptions of the proposed approaches and algorithms will be added, along with additional features and extended capabilities to enhance the system's functionality.

<img src="docs/media/logo.webp" alt="Project Logo" width="600"/>  
---

<img src="docs/media/one_control.gif" alt="PX4 Multi-Drone Control Demonstration 1" width="600">

<img src="docs/media/gz_rviz.gif" alt="PX4 Multi-Drone Control Demonstration 2" width="600">

---

<img src="docs/media/thre_drones_gz.jpg" alt="Three Drones in Gazebo" width="600"/>  

<img src="docs/media/thre_drones_up_gz.jpg" alt="Three Drones from Above" width="600"/>  

---

## **Documentation**

For more details on system architecture, components, and usage, refer to the [Main Documentation](docs/main.md).

---

## **Overview**
This project provides a modular and extensible framework for simulating drones in a PX4-based environment. Key components include:
- **Drone controllers** for managing the logic and behavior of drones.
- **G-code implementation** for defining drone trajectories and actions.
- **Dockerized environment** for consistent setup and deployment.
- **Multi-drone message protocols** for inter-drone communication and state management.

---

## **Project Status**
**This project is currently under active development.**  
New features, improvements, and bug fixes are being added regularly. Contributions and feedback are welcome to help refine and expand the system further.

---

## **Features**

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

## **Getting Started**

### **Fast Run**
1. Clone the repository:
   ```bash
   git clone https://github.com/AntonSHBK/px4_multi_drone_sim.git
   cd px4_multi_drone_sim
   ```
2. Build the Docker container:
   ```bash
   docker-compose up --build
   ```
3. Launch the simulation (inside container workspace `/workplace/src`):
   ```bash
   colcon build --packages-select px4_msgs
   colcon build --packages-select multi_drone multi_drone_msg
   ros2 launch multi_drone test_run.launch.py
   ```

---

## **Example G Commands**

### **1. G20: Move to Point**
```bash
ros2 topic pub --once /id_1_x500/in/command_json std_msgs/msg/String "{data: '{
    \"name\": \"G20\",
    \"counter\": 1,
    \"x\": 10.0,
    \"y\": 5.0,
    \"z\": -10.0,
    \"yaw\": 1.57,
    \"velocity\": 2.0,
    \"coordinate_system\": \"global_ENU\"
}'}"
```

### **2. G21: Linear Move**
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
See the [documentation](documentation/main.md). documentation for other commands.

Use these commands to control the drones and implement complex trajectories. Modify parameters as needed for your specific requirements.

### Dependencies

#### **Required:**
1. **ROS 2 Humble** (on Ubuntu 22.04)  
   Install via [official guide](https://docs.ros.org/en/humble/Installation.html).

2. **PX4 v1.14**  
   Clone and checkout:
   ```bash
   git clone https://github.com/PX4/PX4-Autopilot.git
   cd ./PX4-Autopilot
   git checkout v1.14.4
   git submodule sync --recursive
   git submodule update --init --recursive
   ```

3. **px4_msgs v1.14**  
   ```bash
    git clone https://github.com/PX4/px4_msgs.git
    cd px4_msgs
    git checkout release/1.14
   ```

4. **Docker**  
   Install via [Docker guide](https://docs.docker.com/get-docker/).

5. **Python Libraries**  
   ```bash
   pip install scipy numpy
   ```

---

### **Optional:**
1. **Terminal Emulator:**  
   `gnome-terminal`, `xterm`, or `konsole` for PX4 SITL.  

2. **X Server for Docker Visualization:**  
   Use XLaunch (Windows), XQuartz (macOS), or `xhost` on Linux for GUI support in Docker.  
   Enable X11 forwarding:  
   ```bash
   xhost +local:docker
   ```
---

## Usage

### Fast Commands
Refer to `fast_commands.md` for a list of quick commands and their descriptions.

---

## Contributing
We welcome contributions! Feel free to open issues or submit pull requests.

---

## Citation

```
@misc{px4_multi_drone_sim,
  author       = {Anton Pisarenko},
  title        = {PX4 Multi-Drone Simulation Project},
  year         = {2024},
  month        = {December},
  howpublished = {\url{https://github.com/AntonSHBK/px4_multi_drone_sim}},
  note         = {Accessed: 2024-12-24}
}
```


## Contacts

For questions or support, please contact:

- **Name**: Pisarenko Anton
- **Email**: anton42@yandex.ru
- **Telegram**: [antonSHBK](https://t.me/antonSHBK)
