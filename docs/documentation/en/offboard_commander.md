### offboard_commander.md

#### **Description of `OffboardCommander`**

`OffboardCommander` is a class responsible for controlling the drone in Offboard mode. It allows sending positioning commands, velocity commands, and trajectory waypoints using ROS 2 and interacts with the PX4 autopilot. The main principle of operation involves periodically publishing messages in a format supported by PX4 to manage the drone's state.

---

### **Principles of Offboard Mode**

1. **PX4 Offboard Mode:**
   - PX4 enters Offboard mode when commands are published to specific topics (e.g., `offboard_control_mode` and `trajectory_setpoint`) at a defined frequency.
   - Commands must be sent regularly to maintain Offboard mode.

2. **Main Topics:**
   - **`offboard_control_mode`:** Indicates the type of control (position, velocity, acceleration).
   - **`trajectory_setpoint`:** Sends target trajectory parameters (position, velocity, acceleration, orientation).

3. **Updating Parameters:**
   - Command parameters are updated through the `update` method, where position, velocity, acceleration, yaw, and yaw speed can be specified.

---

### **Key Functions of the Class**

#### **1. Initialization**
The class is initialized by binding it to a base controller and setting up a timer for periodically sending commands.
```python
def __init__(self, controller: X500BaseController, timer_offboard=0.1)
```

- **Arguments:**
  - `controller`: A reference to the base drone controller.
  - `timer_offboard`: The interval in seconds for sending commands (default is 0.1).

- **Publisher Initialization:**
  - `publisher_offboard_mode`: Publishes control mode (position, velocity, acceleration).
  - `publisher_trajectory`: Publishes trajectory setpoints.

#### **2. Updating Parameters**
The `update` method allows setting or modifying command parameters such as position, velocity, yaw, etc.
```python
def update(position=None, velocity=None, acceleration=None, yaw=None, yaw_speed=None, mode=None, system='global_ENU')
```

- **Arguments:**
  - `position`: A point in the format `[x, y, z]`.
  - `velocity`: Velocity in the format `[vx, vy, vz]`.
  - `acceleration`: Acceleration in the format `[ax, ay, az]`.
  - `yaw`: The orientation angle (in radians).
  - `yaw_speed`: The rate of change of yaw.
  - `mode`: The mode for sending commands (`position`, `velocity`, `mixed`).
  - `system`: The coordinate system (`local_NED`, `local_ENU`, `global_ENU`, `global_NED`).

#### **3. Sending Offboard Mode Command**
The `send_offboard_mode` method publishes the control type (position, velocity, acceleration) to the `offboard_control_mode` topic.
```python
def send_offboard_mode(position=False, velocity=False, acceleration=False)
```

- **Arguments:**
  - `position`: Enable position control.
  - `velocity`: Enable velocity control.
  - `acceleration`: Enable acceleration control.

#### **4. Sending Trajectory Setpoint**
The `send_trajectory_setpoint` method publishes the target trajectory parameters (position, velocity, yaw, etc.) to the `trajectory_setpoint` topic.
```python
def send_trajectory_setpoint()
```

- **Published Data:**
  - Position.
  - Velocity (if specified).
  - Acceleration (if specified).
  - Yaw and yaw_speed.

#### **5. Activation and Deactivation**
- `activate()`: Activates the periodic sending of commands.
- `desactivate()`: Stops sending commands.

#### **6. Periodic Command Sending**
The method `_timer_offboard_callback` is called by the timer and sends commands based on the selected mode.
```python
def _timer_offboard_callback()
```

- If the mode is `mixed`: Commands for both position and velocity are sent.
- If the mode is `position`: Commands for position only are sent.
- If the mode is `velocity`: Commands for velocity only are sent.

---

### **Example Usage**

```python
# Initialize OffboardCommander
offboard_commander = OffboardCommander(controller)

# Set target parameters
offboard_commander.update(
    position=[10.0, 5.0, -15.0],
    velocity=[0.5, 0.5, 0.0],
    yaw=1.57,
    mode='mixed'
)

# Activate Offboard mode
offboard_commander.activate()

# After some time, you can change parameters
offboard_commander.update(
    position=[15.0, 10.0, -20.0],
    velocity=None,
    yaw=0.0,
    mode='position'
)

# Deactivate Offboard mode
offboard_commander.desactivate()
```

---

### **Principles of Interaction with PX4**

1. **Regular Command Sending:**
   - To maintain Offboard mode, commands must be sent at the frequency set in `timer_offboard`.

2. **Control Modes:**
   - **`position`:** Specifies the target position.
   - **`velocity`:** Specifies the target velocity.
   - **`mixed`:** Combines position and velocity control.

3. **Flexibility:**
   - The ability to switch between different coordinate systems (ENU, NED, local, and global).

4. **Publishing to Topics:**
   - **`offboard_control_mode`:** Controls the type of control (position, velocity).
   - **`trajectory_setpoint`:** Sends target trajectory parameters.