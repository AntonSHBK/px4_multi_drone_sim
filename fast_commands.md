# Build project

```bash
colcon build --packages-select multi_drone multi_drone_msg


```

# Run

```bash
ros2 launch multi_drone test_gazebo.launch.py

ros2 launch multi_drone test_multi_drone_run.launch.py

ros2 launch multi_drone test_run.launch.py
```