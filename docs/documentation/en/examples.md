### examples.md

#### **Package Usage Examples**

This document contains examples of using various components of the package for drone control. These examples cover basic scenarios such as launching Gazebo and setting up simulations, executing movement commands, and controlling drones.

---

### **1. Launching Gazebo Simulation**

This example demonstrates how to launch a Gazebo simulation using custom models and worlds. The main script configures the path to the models and world, then launches Gazebo.

#### **Code:**
```python
#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare

from multi_drone.utils.methods import load_yaml_params
from multi_drone.scripts.gazebo_server import run_simulation_gazebo


def generate_launch_description():
    
    # Define the path to the parameters file
    pkg_share = FindPackageShare('custom_models').find('custom_models')
    params_file = os.path.join(pkg_share, 'config', 'initial_params.yml')

    # Load parameters from the YAML file
    params: dict = load_yaml_params(params_file)
    default_params: dict = params.get('default_params', {})

    world_file = default_params.get('world_file', 'default')
    github_repo = default_params.get('github_repo', '~/PX4-gazebo-models')
    
    # Launch Gazebo with parameters
    gazebo_server = run_simulation_gazebo(
        world=world_file,
        px4_model_store=github_repo,
        custom_model_store=pkg_share,
        custom_model_store_other=[],
        headless=False,
        overwrite=False,
        gz_ip=None,
        gz_partition=None
    )

    return LaunchDescription([gazebo_server])

if __name__ == "__main__":
    generate_launch_description()
```

#### **Explanation:**
1. **Loading Parameters from YAML:**
   - Simulation parameters such as the world file (`world_file`) and model repository (`github_repo`) are loaded from the `initial_params.yml` file.
2. **Setting Model Path:**
   - Models are loaded from the custom directory (`custom_model_store`) and the standard PX4 repository.
3. **Launching Simulation:**
   - The `run_simulation_gazebo` function is used to launch Gazebo with the specified parameters.

---

### **2. Launching Drones with Custom Controller**

This section demonstrates how to launch multiple drones using a custom controller. The custom controller extends the base `X500Controller` to add support for custom commands.

---

#### **Code:**

```python
#!/usr/bin/env python

import os
import logging

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

from multi_drone.utils.methods import load_yaml_params
from multi_drone.scripts.runner import launch_robot, get_microxrce_agent_exec

def generate_launch_description():
    
    pkg_multirobots = get_package_share_directory('multirobots')
    params_file = os.path.join(pkg_multirobots, 'config', 'drone_config.yml')
    robots_config = load_yaml_params(params_file)

    launch_descriptions = []

    # Add node to launch MicroXRCEAgent
    microxrce_agent = get_microxrce_agent_exec(udp='udp4', port='8888')
    launch_descriptions.append(microxrce_agent)

    # Loop through each robot in the configuration
    for robot in robots_config['robots']:
        
        logging.info(f"Launching drone {robot['drone_id']} of type {robot['drone_type']}")

        launch_descriptions.extend(
            launch_robot(
                drone_id=robot['drone_id'],
                drone_type=robot['drone_type'],
                spawn_position=robot['position'],
                px4_autostart=robot['px4_autostart'],
                px4_dir="/workspace/src/PX4-Autopilot/",
                terminal='bash',
                package_name=robot['package_name'],
                controller_script=robot['controller_script'],
                additional_params={}
            )
        )

    return LaunchDescription(launch_descriptions)

if __name__ == '__main__':
    generate_launch_description()
```

---

#### **Explanation:**

1. **Drone Configuration File:**
   - The `drone_config.yml` file describes the parameters for each drone:
     ```yaml
     robots:
       - drone_id: 1
         drone_type: x500
         position: [0, 0, 0, 0, 0, 0]
         px4_autostart: 4001
         package_name: multirobots
         controller_script: custom_x500.py
       - drone_id: 2
         drone_type: x500
         position: [5, 5, 0, 0, 0, 0]
         px4_autostart: 4002
         package_name: multirobots
         controller_script: custom_x500.py
     ```

2. **Launching MicroXRCEAgent:**
   - This component ensures communication between PX4 and ROS 2.

3. **Launching Drones:**
   - For each drone in the configuration, the `launch_robot` function is called, which:
     - Launches PX4.
     - Launches a ROS 2 node with the custom controller.

---

#### **Creating a Custom Controller**

The custom controller `CustomX500Controller` inherits from `X500Controller` and adds the custom `G101` command.

```python
#!/usr/bin/env python3

import rclpy
from multi_drone.controllers.x500.x500 import X500Controller
from multirobots.scripts.move_commands.x500.example import TestCustomCommand

class CustomX500Controller(X500Controller):
        
    def __init__(
        self,
        drone_id:int=1,
        drone_type:str='x500',
        default_position=[0.0,0.0,0.0],
        default_orientation=[0.0,0.0,0.0]):  
                                          
        super().__init__(
            drone_id=drone_id, 
            drone_type=drone_type, 
            default_position=default_position,
            default_orientation=default_orientation           
        )   
        
        self.g_code_commander.add_command_class('G101', TestCustomCommand)     
       
def main():    
    rclpy.init()     
    control_node = CustomX500Controller()
    
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        control_node.get_logger().info("Stopping the controller.")
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

#### **Creating a Custom Command**

The custom `G101` command describes the drone's movement, for example, in a spiral pattern.

```python
from multi_drone.move_commands.x500.g_code.base_move_command import BaseMoveGCommand

class TestCustomCommand(BaseMoveGCommand):
    """
    Command for drone movement in a spiral pattern.
    """
    def __init__(self, params):
        super().__init__("G101")
        self.params = params

    def update_targets_positions(self):
        # Example logic to create target points for spiral movement
        pass

    def to_dict(self) -> dict:
        return {"name": self.name, "params": self.params}

    @classmethod
    def from_dict(cls, data: dict):
        return cls(params=data.get("params"))

    def __repr__(self):
        return f"TestCustomCommand(params={self.params})"
```

---

#### **Launching**

1. **Drone Configuration File:**
   Ensure the custom controller (`custom_x500.py`) is specified in the `drone_config.yml` file.

2. **Launch Command:**
   ```bash
   ros2 launch multirobots launch_drones.launch.py
   ```