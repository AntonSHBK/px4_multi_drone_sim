### drone_states.md

#### **Drone Status System**

The drone status system is implemented using the State design pattern. Each drone status describes a specific behavior, allowing for efficient management of complex state transitions. This approach ensures modularity, ease of extension, and improved code readability.

---

### **1. General Overview of States**

States are classes that manage the drone's behavior depending on the current task or stage of execution. Each state is responsible for:
- Initializing specific logic.
- Performing operations associated with the state.
- Completing or transitioning to another state.

---

### **2. Main States**

#### **IdleState (Idle Mode)**
- Description:
  - The drone is in an idle state, not performing any active tasks.
  - This is the initial state of the drone after power-up.
- Main Functions:
  - Waiting for a command to transition to another state.
  - Checking system parameters before arming.
- Example Usage:
  ```python
  drone.set_state(IdleState())
  ```

#### **ArmingState (Arming)**
- Description:
  - The drone is in the process of preparing for flight.
  - The motors are started, and safety checks are performed.
- Main Functions:
  - Sending an arming command to PX4.
  - Checking the successful completion of the arming process.
- Example Usage:
  ```python
  drone.set_state(ArmingState())
  ```

#### **TakeoffState (Takeoff)**
- Description:
  - The drone performs a vertical takeoff to a specified height.
- Main Functions:
  - Setting the target height.
  - Transitioning to the position hold mode after takeoff.
- Example Usage:
  ```python
  drone.set_state(TakeoffState(target_height=10.0))
  ```

#### **LoiterState (Loitering)**
- Description:
  - The drone hovers in place, maintaining stable positioning.
- Main Functions:
  - Position and orientation control.
  - Maintaining communication with PX4 to update parameters.
- Example Usage:
  ```python
  drone.set_state(LoiterState())
  ```

#### **OffboardState (Offboard Mode)**
- Description:
  - The drone is controlled by external commands through the Offboard mode.
- Main Functions:
  - Receiving waypoints and velocities.
  - Updating trajectory parameters in real-time.
- Example Usage:
  ```python
  drone.set_state(OffboardState())
  ```

#### **LandingState (Landing)**
- Description:
  - The drone performs a smooth landing to a specified location.
- Main Functions:
  - Controlling the descent rate.
  - Checking for successful completion of the landing.
- Example Usage:
  ```python
  drone.set_state(LandingState())
  ```

#### **DisarmState (Disarming)**
- Description:
  - The drone shuts down the motors and completes any active tasks.
- Main Functions:
  - Sending a disarming command to PX4.
  - Checking the completion of the disarming process.
- Example Usage:
  ```python
  drone.set_state(DisarmState())
  ```

---

### **3. Transition Logic Between States**

State transitions are managed by the logic within each state. For example:
- **Idle → Arming:** After receiving the arm command.
- **Arming → Takeoff:** After successful arming and receiving the takeoff command.
- **Takeoff → Loiter:** After reaching the target altitude.
- **Loiter → Offboard:** When Offboard mode is activated.
- **Offboard → Landing:** After the mission completion command.
- **Landing → Disarm:** After a successful landing.

Each state checks the conditions for completion and triggers a transition to the next state when necessary.

---

### **4. Benefits of the Status System**

1. **Modularity:**
   - Each state is isolated and responsible for its own logic.
   - Adding new states (e.g., mission planning) is simplified.

2. **Ease of Testing:**
   - The logic of each state can be tested independently from other states.

3. **Improved Readability:**
   - Separating the control logic into levels makes it easier to understand the task execution process.

4. **Flexibility:**
   - It is easy to change or add new transition scenarios between states.