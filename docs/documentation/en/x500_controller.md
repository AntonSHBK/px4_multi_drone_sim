### x500_controller.md

#### **Description of X500 Controllers**

The X500 controllers (`X500BaseController` and `X500Controller`) are designed to control X500 quadcopters through interaction with the PX4 autopilot. They extend the functionality of the base `BaseDroneController` class, providing specialized methods and logic for working with this type of drone.

---

### **1. X500BaseController**

`X500BaseController` is an extension of the base `BaseDroneController` class, tailored for X500 drones. Key tasks include:
- Managing the drone's state (arming, takeoff, landing, and other operations).
- Handling high-level commands.
- Interfacing with PX4 via MAVROS.

#### **Key Features**
1. **Support for Control Commands:**
   - Executing movement commands.
   - Updating drone state.
   - Managing modes (e.g., Offboard).

2. **Drone Parameters:**
   - All operational parameters are defined in the `X500Params` class, which provides convenient access to configuration.

3. **Integration with OffboardCommander:**
   - The Offboard mode is handled using `OffboardCommander`, which manages the transmission of trajectories and velocities.

---

### **2. X500Controller**

`X500Controller` extends `X500BaseController`, adding support for handling custom G-codes and high-level commands. The main goal is to provide an interface for executing action sequences, including handling complex scenarios.

#### **Key Features**
1. **Command Queue:**
   - Handles JSON-based commands encoded in G-code format.
   - Supports executing multiple sequential tasks.

2. **Command Completion Check:**
   - Monitors the completion of the current command before transitioning to the next.
   - Integrates with the drone's states to check the completion of tasks.

3. **G-code Interface:**
   - Uses an extendable approach with basic commands (`BaseGCommand`) to implement new scenarios.

---

### **3. X500Params**

`X500Params` is a class for storing X500 drone parameters. It contains:
- **Arming Parameters:**
  - `arm_message`: Arming state (True/False).
  - `arm_state`: Current arming state.
- **Flight Parameters:**
  - Maximum speeds.
  - Takeoff height.
  - Tilt angle limits.
- **PX4 Interaction Parameters:**
  - Command update frequencies.
  - Timeout for various operations.

#### **Example Usage**
```python
params = X500Params()
params.arm_message = True
params.max_speed = 5.0
params.takeoff_height = 10.0
```

---

### **4. OffboardCommander**

`OffboardCommander` is a specialized class for controlling the drone in Offboard mode. Its primary tasks include:
- Publishing trajectories and velocities in Offboard mode.
- Updating target points for movement.
- Maintaining periodic command sending to prevent the mode from being deactivated.

#### **Key Features**
1. **Trajectory Updates:**
   - The `update_targets` method allows setting new coordinates or velocities.

2. **Publishing Commands:**
   - Uses ROS 2 topics to send target position or velocity values.

3. **Frequency Control:**
   - Supports continuous command updates at a set frequency for stable PX4 operation.

#### **Example Usage**
```python
from x500_controller import OffboardCommander

# Initialize OffboardCommander
offboard_commander = OffboardCommander(controller)

# Set a new trajectory
offboard_commander.update_targets(x=10.0, y=5.0, z=15.0, yaw=1.57)

# Publish the trajectory
offboard_commander.publish_target()
```

---

### **Architecture and Interaction**

1. **X500BaseController:**
   - Implements the general drone control logic.
   - Provides methods for basic operations (takeoff, landing, position hold).

2. **X500Controller:**
   - Adds handling of high-level G-codes.
   - Supports expanding functionality through custom commands.

3. **OffboardCommander:**
   - Manages Offboard mode and interacts with PX4.
   - Provides precise control of trajectory and velocities.

4. **X500Params:**
   - Consolidates drone parameters into one class for easy access.