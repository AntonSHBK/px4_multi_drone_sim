### gazebo_launcher.md

#### **Description of `gazebo_server.py` Script**

The `gazebo_server.py` script is designed for automatic setup and launching of Gazebo simulations using custom models and worlds. It allows loading standard and custom models, managing their placement, and running simulations with the necessary parameters, including support for headless mode.

---

### **Key Functions**

#### **1. `run_simulation_gazebo`**
This function is responsible for:
- Loading and preparing Gazebo models.
- Setting up directories for models and worlds.
- Launching the Gazebo simulation with a specified world and parameters.

##### **Function Arguments**
- `world` (str): The name of the world file for simulation (without the extension, e.g., `"default"`).
- `gz_partition` (str): Partitioning Gazebo resources for distributed environments.
- `gz_ip` (str): The IP address for the network interface.
- `model_download_source` (str): URL for downloading Gazebo models.
- `px4_model_store` (str): Directory for storing standard PX4 models.
- `custom_model_store` (str): Directory for custom models.
- `custom_model_store_other` (List[str]): Additional paths for custom models.
- `overwrite` (bool): Flag for overwriting existing models.
- `headless` (bool): Flag for running in headless mode (without graphical interface).

##### **Return Value**
- `ExecuteProcess`: An object representing the process of running Gazebo.

---

### **Workflow Description**

#### **1. Model Preparation**
- Checks for models in the `px4_model_store` directory.
- If models are missing or the `overwrite` flag is set, downloads them from the specified source (`model_download_source`).
- Unzips the archive and organizes files into the `models` and `worlds` directories.

#### **2. Path Setup**
- Sets the `GZ_SIM_RESOURCE_PATH` environment variable, which defines the paths to the models and worlds for the simulation.

#### **3. Command Formation**
- Forms the command for launching the simulation:
  - Specifies the world file path.
  - Adds parameters for headless mode (`-s`) or network settings (`gz_partition`, `gz_ip`).

#### **4. Simulation Launch**
- Uses `ExecuteProcess` to run Gazebo with the specified parameters.

---

### **Key Variables**

1. **`px4_model_store`:**
   - Local directory for storing PX4 models.
   - Default: `~/simulation-gazebo`.

2. **`model_download_source`:**
   - URL for downloading standard PX4 models.
   - Default: `https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip`.

3. **`GZ_SIM_RESOURCE_PATH`:**
   - List of paths for Gazebo models and worlds.

4. **`gz_cmd`:**
   - Command for launching the simulation. It is formed based on the provided parameters.

---

### **Usage**

1. **Launch Simulation with Default Settings:**
   ```bash
   python3 gazebo_server.py
   ```

2. **Example Launch of a Custom World with Additional Settings:**
   ```python
   from gazebo_server import run_simulation_gazebo

   run_simulation_gazebo(
       world="custom_world",
       gz_partition="simulation_partition",
       gz_ip="192.168.1.100",
       model_download_source="https://example.com/custom_models.zip",
       px4_model_store="/home/user/gazebo_models",
       custom_model_store="/home/user/custom_worlds",
       custom_model_store_other=["/opt/additional_models", "/usr/share/models"],
       overwrite=True,
       headless=True
   )
   ```

3. **Configuring Custom Models and Worlds:**
   - Specify the directory for custom models via the `custom_model_store` argument.
   - Add additional paths via `custom_model_store_other`.

---

### **Example Gazebo Launch Command**

After running the script, the following command will be formed:
```bash
gz sim -r ~/simulation-gazebo/worlds/custom_world.sdf -s
```
Where:
- `-r`: Specifies the path to the world file.
- `-s`: Launches in headless mode (if the `headless` flag is set).

---

### **Advantages of Using This Script**

1. **Automation of Setup:**
   - The script automatically downloads, unzips, and organizes models, simplifying the simulation setup.

2. **Support for Custom Models:**
   - Easy integration of custom models and worlds.

3. **Scalability:**
   - Supports resource partitioning and network interfaces for distributed systems.

4. **Development Convenience:**
   - Running Gazebo in headless mode saves resources during testing.

---

### **Conclusion**

The `gazebo_server.py` script provides a convenient interface for working with Gazebo, automating the setup and launch of simulations. It supports flexible configuration and scalability, making it an ideal tool for working with PX4 and ROS-based projects.