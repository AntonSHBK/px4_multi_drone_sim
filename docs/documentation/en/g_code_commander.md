### g_code_commander.md

#### **Description of `X500Commander`**

The `X500Commander` class implements the processing and execution of high-level G-code commands for the X500 drone. It extends the functionality of the base `DroneCommander` class, providing tools for managing command queues, sequential execution, and handling special scenarios.

---

### **1. Base Class `DroneCommander`**

`DroneCommander` is an abstract class that provides the structure for creating commanders. Key features include:
- Managing the command queue.
- Processing incoming JSON commands.
- Monitoring the state of the active command.

---

### **2. Key Functions of `X500Commander`**

#### **2.1 Initialization**

The constructor of the class initializes the following components:
- **Registering All Commands:** Using the `g_code_module`, all classes inheriting from `BaseGCommand` are registered, allowing for dynamic addition of new commands.
- **Subscribing to Incoming Commands:** Through the ROS subscription to the topic `<{prefix_name}>/in/command_json`, JSON commands are received.
- **Command Timer:** Using a ROS timer, commands from the queue are processed at a specified frequency.

```python
def __init__(self, controller: "X500Controller", timer_execution=0.1)
```

---

#### **2.2 Processing JSON Commands**

1. **Receiving a JSON Command:**
   The method `command_json_callback` processes incoming ROS messages by decoding the JSON and passing the data to the `process_incoming_command` method.
   ```python
   def command_json_callback(self, msg: String)
   ```

2. **Processing Incoming Data:**
   The method `process_incoming_command` extracts the command name (`name`) and finds the corresponding command class from the registered commands.
   ```python
   def process_incoming_command(self, data: dict)
   ```

3. **Creating the Command:**
   The static method `from_dict` of the command class is used to convert the JSON data into a command instance.
   ```python
   command_class.from_dict(data)
   ```

4. **Adding to Queue:**
   If the command is not special (e.g., `G0`), it is added to the queue.

---

#### **2.3 Executing Commands**

1. **Command Timer:**
   The method `_timer_execute_commands_callback` is called by the ROS timer and checks the state of the active command or takes the next command from the queue.
   ```python
   def _timer_execute_commands_callback(self)
   ```

2. **Completion Check:**
   If the command is completed (`is_complete`), the `handle_completion` method is called, and the active command is cleared.

3. **Executing the Next Command:**
   If the command queue is not empty, the next command is activated by calling its `safe_execute` method.

---

#### **2.4 Special Commands**

The method `check_special_command` checks if the command is special (e.g., `G0`) and processes it immediately without adding it to the queue.

```python
def check_special_command(self, command: BaseGCommand) -> bool
```

---

### How to Create a Drone Movement Command

#### **General Principle**

A drone movement command is created as a class inheriting from the base class `BaseMoveGCommand`. It describes:
1. **Movement Type** (linear movement, circular trajectory, etc.).
2. **Target Points** (start and end positions).
3. **Movement Parameters** (velocity, orientation `yaw`).

Each command must implement the following key methods:
- `update_targets_positions`: defines the list of target points.
- `execute`: sends commands to the drone to perform the movement.
- `is_complete`: checks if the current phase or the entire command is completed.

---

### **Steps to Create a Command**

#### **1. Inherit the Class from `BaseMoveGCommand`**

The base class provides a basic structure for movement commands:
- Stores the list of target positions (`targets_positions`).
- Defines the general logic for execution and completion checking.

#### **2. Define the Command Parameters**

Example: For linear movement (`G21_LinearMove`), you need to specify the start and end points, velocity, and orientation `yaw`.

```python
class G21_LinearMove(BaseMoveGCommand):
    """
    Command for linear drone movement between two points.
    """
    def __init__(self, 
                 counter: int = 0,
                 start_point: Optional[List[float]] = None,
                 end_point: Optional[List[float]] = None,
                 velocity: float = 1.0,
                 yaw: Optional[float] = None,
                 coordinate_system: Literal[
                    "local_NED", "local_ENU", 'global_ENU', 'global_NED'
                 ] = 'global_ENU',
                 current_step: int = 0):
        super().__init__("G21", counter, coordinate_system, current_step)
        self.start_point = np.array(start_point) if start_point else None
        self.end_point = np.array(end_point) if end_point else None
        self.velocity = velocity
        self.yaw = yaw
        self.update_targets_positions()
```

#### **3. Implement `update_targets_positions` Method**

This method defines the list of target positions. For linear movement, it involves the start and end points.

```python
def update_targets_positions(self):
    self.targets_positions.clear()

    if self.start_point is not None:
        self.targets_positions.append(
            self.Position(
                x=self.start_point[0],
                y=self.start_point[1],
                z=self.start_point[2],
                velocity=self.velocity,
                yaw=self.yaw
            )
        )

    if self.end_point is not None:
        self.targets_positions.append(
            self.Position(
                x=self.end_point[0],
                y=self.end_point[1],
                z=self.end_point[2],
                velocity=self.velocity,
                yaw=self.yaw
            )
        )
```

#### **4. Implement Serialization and Deserialization Methods**

These methods allow saving and loading command parameters from a dictionary, for example, for transmission via JSON.

```python
def to_dict(self) -> dict:
    base_dict = super().to_dict()
    base_dict.update({
        "start_point": self.start_point.tolist() if self.start_point is not None else None,
        "end_point": self.end_point.tolist() if self.end_point is not None else None,
        "velocity": self.velocity,
        "yaw": self.yaw,
    })
    return base_dict

@classmethod
def from_dict(cls, data: dict):
    return cls(
        counter=data.get("counter", 0),
        start_point=data.get("start_point"),
        end_point=data.get("end_point"),
        velocity=data.get("velocity", 1.0),
        yaw=data.get("yaw"),
        coordinate_system=data.get("coordinate_system", 'global_ENU'),
        current_step=data.get("current_step", 0)
    )
```

---

### **Example of Using the Command**

1. **Create a Command:**
   ```python
   from multi_drone.move_commands.x500.g_code import G21_LinearMove

   command = G21_LinearMove(
       start_point=[0.0, 0.0, 0.0],
       end_point=[10.0, 5.0, -15.0],
       velocity=2.0,
       yaw=1.57
   )
   print(command)
   ```

2. **Send the Command via JSON:**
   ```python
   import json

   json_data = json.dumps(command.to_dict())
   print(json_data)
   ```

3. **Load the Command from JSON:**
   ```python
   loaded_command = G21_LinearMove.from_dict(json.loads(json_data))
   print(loaded_command)
   ```

4. **Add the Command to `X500Commander`:**
   ```python
   commander.process_incoming_command(command.to_dict())
   ```

---

### **Example JSON Command**

```json
{
    "name": "G21",
    "counter": 1,
    "start_point": [0.0, 0.0, 0.0],
    "end_point": [10.0, 5.0, -15.0],
    "velocity": 2.0,
    "yaw": 1.57,
    "coordinate_system": "global_ENU",
    "current_step": 0
}
```

---

### **Key Parameters**

1. **`name` (str):** The command name (e.g., `G21`).
2. **`start_point` (list):** The start point `[x, y, z]`.
3. **`end_point` (list):** The end point `[x, y, z]`.
4. **`velocity` (float):** The movement velocity (in m/s).
5. **`yaw` (float):** The orientation angle (in radians).
6. **`coordinate_system` (str):** The coordinate system (`local_NED`, `local_ENU`, `global_ENU`, `global_NED`).