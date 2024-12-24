### drone_info_msg.md

#### **Description of `DroneInformMsg` Messages**

`DroneInformMsg` messages are used to publish the current information about the drone's state and parameters. They allow other system modules to receive data about the drone's current position, target point, and operational parameters. These messages are broadcasted on the topic:

```
<{prefix_name}>/out/inform_of_drone
```

where `{prefix_name}` is a unique prefix for the drone's name, defined during initialization.

---

### **Message Structure of `DroneInformMsg`**

The message consists of the following fields:

1. **`params` (type: `DroneParamsMsg`):**
   - Contains the parameters of the drone's current state.
   - Fields:
     - `nav_state` (uint8): Current navigation state.
     - `arm_state` (uint8): Arm state (armed/unarmed).
     - `offboard_mode` (bool): Whether the Offboard mode is active.
     - `flight_check` (bool): Flight readiness check.
     - `arm_message` (bool): Indicates if an arm message was sent.
     - `failsafe` (bool): Indicates if a failsafe mode is active.
     - `landing` (bool): Indicates whether the drone is landing.

2. **`current_position` (type: `LocalAndGlobalCoordinatesMsg`):**
   - The drone's current position in various coordinate systems.
   - Fields:
     - `local_enu`, `local_ned`, `global_enu`, `global_ned` (type: `CoordinateDataMsg`): Coordinates in ENU and NED systems, both local and global.

3. **`target_position` (type: `LocalAndGlobalCoordinatesMsg`):**
   - The target position of the drone.
   - Similar to `current_position`, it contains coordinates in various systems.

---

### **Message Structure Description**

1. **`CoordinateDataMsg`:**
   - Describes geometric information about position and velocity.
   - Fields:
     - `position` (type: `geometry_msgs/Vector3`): Coordinates (x, y, z).
     - `velocity` (type: `geometry_msgs/Vector3`): Linear velocities along axes (vx, vy, vz).
     - `yaw` (type: `float64`): Heading angle.

2. **`LocalAndGlobalCoordinatesMsg`:**
   - Represents local and global coordinates.
   - Fields:
     - `local_enu`, `local_ned` (type: `CoordinateDataMsg`): Local coordinates in ENU and NED systems.
     - `global_enu`, `global_ned` (type: `CoordinateDataMsg`): Global coordinates in ENU and NED systems.

3. **`DroneParamsMsg`:**
   - Represents parameters of the drone's current state.
   - The fields are described above.

---

### **Message Topics**

1. **Publishing Drone State:**
   - Topic: `<{prefix_name}>/out/inform_of_drone`
   - Message Type: `DroneInformMsg`
   - Updated periodically according to the `timer_inform_of_drone` parameter (default is 0.5 seconds).

2. **Subscribing to Autopilot Status:**
   - Topic: `<{prefix_px}>/fmu/out/vehicle_status`
   - Message Type: `VehicleStatus`
   - Used to update the drone's parameters (`params`) based on PX4 autopilot data.

---

### **Example of Message Publishing**

The method `publish_inform_of_drone` is responsible for collecting and publishing the drone's data:
```python
def publish_inform_of_drone(self):
    msg = DroneInformMsg()
    msg.params = self.params.to_msg()  # Drone parameters
    msg.current_position = self.current_position.to_msg()  # Current position
    msg.target_position = self.target_position.to_msg()  # Target position
    self.publisher_inform_of_drone.publish(msg)
```

This method:
1. Collects data from the drone's parameters (`self.params`).
2. Converts the current and target position data into message format.
3. Publishes the message to the topic.

---

### **Example Usage of Messages**

1. **Subscribing to Messages:**
   In another ROS node, you can subscribe to drone data:
   ```python
   def drone_info_callback(msg: DroneInformMsg):
       print("Current position:", msg.current_position.local_enu.position)

   node.create_subscription(
       DroneInformMsg,
       '<drone_name>/out/inform_of_drone',
       drone_info_callback,
       qos_profile_unreliable
   )
   ```

2. **Analyzing Parameters:**
   The received data can be used to monitor the drone's state:
   ```python
   if msg.params.arm_state == 1:
       print("The drone is armed.")
   ```