### G_code_command.md

#### **How to Send G-codes to the Appropriate Drones**

G-code commands are used to control drone actions. These commands are transmitted in JSON format via ROS 2 topics and are interpreted by drone commanders. This guide describes the steps for sending standard and custom G-code commands.

---

### **Main Principle**

1. **G-codes** are JSON commands that contain:
   - Command name (`name`).
   - Command counter (`counter`).
   - Command parameters depending on its type (e.g., position, speed).
   
2. **Command topics:**
   - Each command is sent to the corresponding drone topic, e.g., `/id_1_x500/in/command_json`.
   
3. **Format used:**
   - Commands must be in JSON format.

---

### **Examples of Sending G-commands**

### **Basic Commands**

#### **1. G0: Reset Commands**
The `G0` command stops the current command execution, clears the command queue, and returns the drone to a safe state.

**Example JSON:**
```json
{
    "name": "G0",
    "counter": 0
}
```

**Description:**
- Completely stops active commands.
- Clears the command queue.

**Sending Example:**
```bash
ros2 topic pub --once /id_1_x500/in/command_json std_msgs/msg/String "{data: '{\"name\": \"G0\", \"counter\": 0}'}"
```

#### **2. Sending Basic Commands**

These commands control basic drone actions: arming, takeoff, landing, etc.

##### **G-codes: G1-G6**
```bash
#!/bin/bash

# Topic for sending commands
TOPIC="/id_1_x500/in/command_json"

# G1: Arming
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G1\",
    \"counter\": 1
}'}"

# G2: Disarming
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G2\",
    \"counter\": 2
}'}"

# G3: Takeoff to 2.0 meters
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G3\",
    \"counter\": 3,
    \"altitude\": 2.0
}'}"

# G4: Landing
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G4\",
    \"counter\": 4
}'}"

# G5: Hold position
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G5\",
    \"counter\": 5
}'}"

# G6: Enable Offboard mode
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G6\",
    \"counter\": 6
}'}"

echo "Basic commands sent."
```

#### **General Recommendations**

1. **Check Topics:**
   - Before sending commands, ensure the drone topic is active:
     ```bash
     ros2 topic list
     ```

2. **JSON Validator:**
   - Use online JSON validators to check the syntax of commands.

3. **Coordinate System Specification:**
   - `coordinate_system` can be one of:
     - `global_ENU`
     - `local_NED`
     - `local_ENU`
     - `global_NED`

---

#### **1. G20: Move to Point**
The `G20_MoveToPoint` command is used to move the drone to a specified point with an optional orientation (`yaw`) and velocity.

**Example JSON:**
```json
{
    "name": "G20",
    "counter": 1,
    "x": 10.0,
    "y": 5.0,
    "z": -10.0,
    "yaw": 1.57,
    "velocity": 2.0,
    "coordinate_system": "global_ENU"
}
```

**Parameter Description:**
- `x`, `y`, `z`: Target coordinates.
- `yaw`: Orientation angle (in radians).
- `velocity`: Movement speed (m/s).
- `coordinate_system`: Coordinate system (`global_ENU`, `local_ENU`, etc.).

**Sending Example:**
```bash
#!/bin/bash

# Topic for sending command
TOPIC="/id_1_x500/in/command_json"

# G20: MoveToPoint
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G20\",
    \"counter\": 1,
    \"x\": 10.0,
    \"y\": 5.0,
    \"z\": -10.0,
    \"coordinate_system\": \"global_ENU\"
}'}"

echo "G20_MoveToPoint command sent."
```

---

#### **2. G21: Linear Move**
The `G21_LinearMove` command defines linear motion between two points.

**Example JSON:**
```json
{
    "name": "G21",
    "counter": 2,
    "start_point": [0.0, 0.0, 0.0],
    "end_point": [5.0, 10.0, -10.0],
    "velocity": 1.5,
    "yaw": 1.57,
    "coordinate_system": "global_ENU"
}
```

**Parameter Description:**
- `start_point`: Start point `[x, y, z]`.
- `end_point`: End point `[x, y, z]`.
- `velocity`: Movement speed (m/s).
- `yaw`: Orientation angle (in radians).
- `coordinate_system`: Coordinate system.

**Sending Example:**
```bash
#!/bin/bash

# Topic for sending command
TOPIC="/id_1_x500/in/command_json"

# G21: LinearMove
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G21\",
    \"counter\": 2,
    \"start_point\": [0.0, 0.0, 0.0],
    \"end_point\": [5.0, 10.0, -10.0],
    \"velocity\": 1.5,
    \"coordinate_system\": \"global_ENU\"
}'}"

echo "G21_LinearMove command sent."
```

---

#### **3. G22: Circular Trajectory**
The `G22_CircularTrajectory` command defines movement along an arc.

**Example JSON:**
```json
{
    "name": "G22",
    "counter": 3,
    "start_point": [0.0, 0.0, 0.0],
    "end_point": [5.0, 10.0, -10.0],
    "radius": 10.0,
    "direction": "CCW",
    "points_count": 20,
    "coordinate_system": "global_ENU"
}
```

**Parameter Description:**
- `start_point`: Start point `[x, y, z]`.
- `end_point`: End point `[x, y, z]`.
- `radius`: Trajectory radius (m).
- `direction`: Direction (`CW` — clockwise, `CCW` — counterclockwise).
- `points_count`: Number of points along the arc.
- `coordinate_system`: Coordinate system.

**Sending Example:**
```bash
#!/bin/bash

# Topic for sending command
TOPIC="/id_1_x500/in/command_json"

# Sending G22_CircularTrajectory command
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G22\",
    \"counter\": 3,
    \"start_point\": [5.0, 10.0, 20.0],
    \"end_point\": [10.0, 5.0, 20.0],
    \"radius\": 25.0,
    \"direction\": \"CCW\",
    \"points_count\": 10,
    \"coordinate_system\": \"global_ENU\"
}'}"

echo "G22_CircularTrajectory command sent to topic $TOPIC."
```

---

#### **4. G23: Orbit Flight**
The `G23_Orbit` command defines flight along an orbit around a specified point.

**Example JSON:**
```json
{
    "name": "G23",
    "counter": 4,
    "center_point": [0.0, 0.0, 0.0],
    "radius": 5.0,
    "angular_velocity": 0.1,
    "orbit_direction": "CW",
    "yaw_mode": "facing_center",
    "duration": 30,
    "coordinate_system": "global_ENU"
}
```

**Parameter Description:**
- `center_point`: Orbit center `[x, y, z]`.
- `radius`: Orbit radius (m).
- `angular_velocity`: Angular velocity (radians/s).
- `orbit_direction`: Direction (`CW`, `CCW`).
- `yaw_mode`: Orientation mode (`fixed`, `facing_center`).
- `duration`: Duration (seconds).

**Sending Example:**
```bash
#!/bin/bash

# Topic for sending commands
TOPIC="/id_1_x500/in/command_json"

ros2 topic pub --once $TOPIC std_msgs/String "{data: '{
  \"name\": \"G23\",
  \"counter\": 1,
  \"center_point\": [0.0, 0.0, 20.0],
  \"radius\": 5.0,
  \"angular_velocity\": 0.5,
  \"orbit_direction\": \"CCW\",
  \"yaw_mode\": \"facing_center\",
  \"duration\": 30,
  \"velocity\": null,
  \"coordinate_system\": \"global_ENU\"
}'}"
```