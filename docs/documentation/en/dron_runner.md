### dron_runner.md

#### **Description of the `runner.py` Functionality**

The `runner.py` file provides an interface for automatically launching a drone simulation using PX4 and ROS 2. The main functionality includes:
- Creating processes for the PX4 autopilot.
- Creating ROS 2 nodes for drone control.
- Configuring launches for one or more drones.

---

### **Main Classes**

#### **1. PX4Process**
This class is responsible for creating and configuring the PX4 process that simulates the operation of the drone's autopilot.

##### **Constructor Arguments:**
- `drone_id` (int): A unique identifier for the drone.
- `drone_type` (str): The type of the drone (e.g., `'x500'`).
- `spawn_position` (list): The starting position and orientation of the drone in the format `[x, y, z, roll, pitch, yaw]`.
- `px4_autostart` (int): The PX4 autostart code for drone configuration.
- `px4_dir` (str): Path to the PX4 autopilot directory.
- `terminal` (str): Terminal for launching the process (`'gnome-terminal'`, `'xterm'`, `'konsole'`, `'bash'`).

##### **Methods:**
- `create_process() -> ExecuteProcess`: Creates the PX4 process to launch the simulation.

##### **Usage Example:**
```python
px4_process = PX4Process(
    drone_id=1,
    drone_type='x500',
    spawn_position=[0, 0, 0, 0, 0, 0],
    px4_autostart=4001,
    px4_dir="/workspace/src/PX4-Autopilot/",
    terminal='gnome-terminal'
)
px4_execute_process = px4_process.create_process()
```

---

#### **2. ControllerNode**
This class is used to create a ROS 2 node that manages the behavior of the drone.

##### **Constructor Arguments:**
- `package_name` (str): The name of the ROS 2 package containing the controller.
- `controller_script` (str): The name of the node’s executable file.
- `drone_id` (int): A unique identifier for the drone.
- `drone_type` (str): The type of the drone (e.g., `'x500'`).
- `spawn_position` (list): The starting position and orientation of the drone in the format `[x, y, z, roll, pitch, yaw]`.
- `output` (str): The output type (`'log'` or `'screen'`).
- `additional_params` (dict): Additional parameters for the ROS 2 node.

##### **Methods:**
- `create_node() -> Node`: Creates the ROS 2 node with the given parameters.

##### **Usage Example:**
```python
controller_node = ControllerNode(
    package_name='multi_drone',
    controller_script='x500.py',
    drone_id=1,
    drone_type='x500',
    spawn_position=[0, 0, 0, 0, 0, 0],
    output='log'
)
ros2_node = controller_node.create_node()
```

---

#### **3. `launch_robot` Function**
This function combines the PX4 and ROS 2 components into a single list of processes and nodes to launch the simulation for a single drone.

##### **Arguments:**
- `drone_id` (int): The drone's identifier.
- `drone_type` (str): The drone's type.
- `spawn_position` (list): The drone's position and orientation.
- `px4_autostart` (int): PX4 autostart configuration.
- `px4_dir` (str): Path to PX4 Autopilot.
- `terminal` (str): Terminal for launching PX4.
- `package_name` (str): The name of the ROS 2 package.
- `controller_script` (str): The controller script's name.
- `additional_params` (dict): Additional parameters for the ROS 2 node.

##### **Usage Example:**
```python
nodes = launch_robot(
    drone_id=1,
    drone_type='x500',
    spawn_position=[0, 0, 0, 0, 0, 0],
    px4_autostart=4001,
    px4_dir="/workspace/src/PX4-Autopilot/",
    terminal='gnome-terminal',
    package_name='multi_drone',
    controller_script='x500.py',
    additional_params={}
)
```

---

### **Key Functions**

#### **1. `get_microxrce_agent_exec`**
Creates a process to run the `MicroXRCEAgent`, which enables communication between PX4 and ROS 2.

##### **Arguments:**
- `udp` (str): The protocol (default `'udp4'`).
- `port` (str): The port (default `'8888'`).

##### **Usage Example:**
```python
microxrce_agent = get_microxrce_agent_exec(udp='udp4', port='8888')
```

---

### **Example of Full Launch**

```python
from runner import launch_robot

# Launch a single drone with ID=1
nodes = launch_robot(
    drone_id=1,
    drone_type='x500',
    spawn_position=[0, 0, 0, 0, 0, 0],
    px4_autostart=4001,
    px4_dir="/workspace/src/PX4-Autopilot/",
    terminal='gnome-terminal',
    package_name='multi_drone',
    controller_script='x500.py',
    additional_params={}
)

for node in nodes:
    print(f"Launching process: {node}")
```