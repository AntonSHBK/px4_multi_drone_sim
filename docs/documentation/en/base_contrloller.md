### base_controller.md

#### **Description of the Base Controller Class**

The base controller class (`BaseDroneController`) is designed for controlling a drone through interaction with the PX4 autopilot. It implements the core methods and logic that form the foundation for developing advanced controllers. This class handles command processing, message transmission, and synchronization of the drone's state with the PX4 autopilot via ROS 2.

---

#### **Main Features of `BaseDroneController`**

1. **Controller Initialization:**
   - The constructor accepts the following parameters:
     - `drone_id`: a unique identifier for the drone.
     - `drone_type`: the type of drone.
     - `default_position` and `default_orientation`: initial coordinates and orientation.
   - Initialization of ROS topics for publishing commands and subscribing to telemetry data.

2. **QoS Profiles:**
   - Standard QoS profiles are defined:
     - `qos_profile_reliable`: for guaranteed message delivery.
     - `qos_profile_unreliable`: for messages with lower latency but no delivery guarantee.

3. **Logging Methods:**
   - `log_info` and `log_error`: output information about the controller's operation and potential errors.

4. **Geometric Transformations:**
   - Used for transitioning between different coordinate systems (e.g., ENU, NED) and converting orientation data.

---

#### **Geometric Transformations**

The base controller includes functions for handling geometric data such as position, velocity, and orientation. Transformations are necessary for synchronizing data between various coordinate systems and PX4.

- **ENU and NED:**
  - ENU (East-North-Up) is used in ROS.
  - NED (North-East-Down) is the standard system in PX4.
  - The controller's methods transform data between these systems.

- **Transformation Examples:**
  - The `enu_to_ned` method transforms a vector from the ENU coordinate system to the NED system.
  - The `convert_to_FLU` method converts coordinates to the drone's coordinate system (Forward-Left-Up).

These functions use `numpy` and `scipy` libraries for matrix transformations and calculation of angular orientations.

---

#### **Data Classes (BaseData and Derivatives)**

Specialized classes are used to represent position, orientation, and velocity data:

1. **`BaseData`:**
   - The base class for all data types.
   - Contains common methods, such as:
     - Converting data to a dictionary.
     - Comparing objects.

2. **`PositionData`:**
   - Describes the position in space.
   - Attributes:
     - `x`, `y`, `z`: coordinates.
   - Methods:
     - Supports transformations between coordinate systems.

3. **`VelocityData`:**
   - Represents linear and angular velocities.
   - Attributes:
     - `vx`, `vy`, `vz`: linear velocities.
     - `roll_rate`, `pitch_rate`, `yaw_rate`: angular velocities.

4. **`OrientationData`:**
   - Represents the drone's orientation.
   - Attributes:
     - `roll`, `pitch`, `yaw`: orientation angles.
     - Methods for converting between quaternions and Euler angles.

5. **`QuaternionData`:**
   - Stores orientation data in quaternion format.
   - Used for precise calculations and transformations.

---

#### **Features and Benefits**

- **Modularity:**
  - Geometric transformations and data classes are isolated from the main controller, simplifying functionality extension.

- **Versatility:**
  - The controller supports working with different coordinate systems, making it suitable for integration into complex projects.

- **Ease of Use:**
  - An intuitive API for interacting with position, velocity, and orientation data.