## **Сборка проекта**

```bash
colcon build --packages-select multi_drone multi_drone_msg
```

**Описание:**
- Собирает указанные пакеты `multi_drone` и `multi_drone_msg`. 
- Убедитесь, что вы находитесь в корневой директории вашего ROS 2 рабочего пространства перед запуском команды.

---

## **Запуск**

### **1. Запуск тестовой среды Gazebo**
```bash
ros2 launch multi_drone test_gazebo.launch.py
```
- Запускает тестовую симуляцию Gazebo для работы с дроном.

### **2. Запуск нескольких дронов**
```bash
ros2 launch multi_drone test_multi_drone_run.launch.py
```
- Запускает несколько дронов в симуляции, если проект поддерживает работу с несколькими агентами.

### **3. Запуск стандартного контроллера**
```bash
ros2 launch multi_drone test_run.launch.py
```
- Запускает стандартный сценарий управления дроном.

---

## **Прочее**

### **Просмотр топика с информацией о дроне**
```bash
ros2 topic echo /id_1_x500/out/inform_of_drone
```
- Выводит сообщения из топика `/id_1_x500/out/inform_of_drone`.
- Замените `id_1_x500` на ID вашего дрона, если используете другой идентификатор.

---

### Полезные команды ROS 2:

#### Проверка активных нод:
```bash
ros2 node list
```

#### Просмотр всех доступных топиков:
```bash
ros2 topic list
```

#### Информация о топике:
```bash
ros2 topic info /topic_name
```

#### Публикация тестового сообщения:
```bash
ros2 topic pub /topic_name std_msgs/String "data: 'Hello, Drone!'"
```

#### Запуск RQT для визуализации данных:
```bash
rqt
```

## G command test

```bash
#!/bin/bash

ros2 topic pub --once /px4_1/command_json std_msgs/msg/String "{data: '{\"name\": \"G0\", \"counter\": 0}'}"

# Отправка команды G1 (Arm)
ros2 topic pub --once /px4_1/command_json std_msgs/msg/String "{data: '{\"name\": \"G1\", \"counter\": 1}'}"

# Отправка команды G2 (Disarm)
ros2 topic pub --once /px4_1/command_json std_msgs/msg/String "{data: '{\"name\": \"G2\", \"counter\": 2}'}"

# Отправка команды G3 (Takeoff на высоту 2.0 м)
ros2 topic pub --once /px4_1/command_json std_msgs/msg/String "{data: '{\"name\": \"G3\", \"counter\": 3, \"altitude\": 2.0}'}"

# Отправка команды G4 (Landing)
ros2 topic pub --once /px4_1/command_json std_msgs/msg/String "{data: '{\"name\": \"G4\", \"counter\": 4}'}"

# Отправка команды G5 (Loiter)
ros2 topic pub --once /px4_1/command_json std_msgs/msg/String "{data: '{\"name\": \"G5\", \"counter\": 5}'}"

# Отправка команды G6 (Offboard)
ros2 topic pub --once /px4_1/command_json std_msgs/msg/String "{data: '{\"name\": \"G6\", \"counter\": 6}'}"

echo "Команды G1-G6 успешно отправлены на топик /px4_1/command_json."

```

Вот пример **bash-скриптов** для отправки команд `G20_MoveToPoint`, `G21_LinearMove` и `G22_CircularTrajectory` в топик. Предполагается, что используется **ROS 2** и формат JSON.

---

### **1. Команда `G20_MoveToPoint`**
```bash
#!/bin/bash

# Топик для отправки команд
TOPIC="/px4_2/command_json"

# Отправка команды G20_MoveToPoint
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G20\",
    \"counter\": 1,
    \"x\": 0.0,
    \"y\": 0.0,
    \"z\": 20.0,
    \"coordinate_system\": \"global_ENU\"
}'}"

echo "Команда G20_MoveToPoint отправлена в топик $TOPIC."
```

---

### **2. Команда `G21_LinearMove`**
```bash
#!/bin/bash

# Топик для отправки команд
TOPIC="/px4_1/command_json"

# Отправка команды G21_LinearMove
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G21\",
    \"counter\": 2,
    \"start_point\": [0.0, 0.0, 20.0],
    \"end_point\": [5.0, 10.0, 20.0],
    \"coordinate_system\": \"global_ENU\"
}'}"

echo "Команда G21_LinearMove отправлена в топик $TOPIC."
```

---

### **3. Команда `G22_CircularTrajectory`**
```bash
#!/bin/bash

# Топик для отправки команд
TOPIC="/px4_1/command_json"

# Отправка команды G22_CircularTrajectory
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

echo "Команда G22_CircularTrajectory отправлена в топик $TOPIC."


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

ros2 topic pub --once $TOPIC std_msgs/String "{data: '{
  \"name\": \"G24\",
  \"counter\": 2,
  \"center_point\": [0.0, 0.0, 20.0],
  \"radius_start\": 3.0,
  \"radius_end\": 8.0,
  \"height_change\": 10.0,
  \"turns\": 3,
  \"direction\": \"CW\",
  \"points_per_turn\": 10,
  \"velocity\": 2.0,
  \"yaw_mode\": \"facing_center\",
  \"coordinate_system\": \"local_ENU\"
}'}"

```

---

### **Как использовать скрипты**
1. Сохраните каждый скрипт в отдельный файл, например:
   - **`send_g20.sh`** для команды `G20_MoveToPoint`.
   - **`send_g21.sh`** для команды `G21_LinearMove`.
   - **`send_g22.sh`** для команды `G22_CircularTrajectory`.

2. Сделайте файлы исполняемыми:
   ```bash
   chmod +x send_g20.sh send_g21.sh send_g22.sh
   ```

3. Запустите скрипт:
   ```bash
   ./send_g20.sh
   ./send_g21.sh
   ./send_g22.sh
   ```

---

Если нужно добавить параметры или изменить топик, дайте знать! 🚀




### **Заголовок темы**
Как корректно выйти из Offboard режима и перейти в Loiter (NavState = 5)?

Я столкнулся с проблемой при попытке выхода из режима **Offboard** и переключения дрона в режим **Loiter (NavState = 5)**. Я разбираю пример и руководства [px4_ros_com]https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard_py/offboard_control.py.

#### **Моя конфигурация**
- **Язык:** Python
- **Платформа:** PX4
- **Версия прошивки:** v1.13

#### **Что я делаю**
1. Выполняю команду на взлёт:
   ```python
   self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5.0)
   ```
   После этого дрон переходит в режим зависания (предполагаю, что это **Loiter**).

2. Переключаю дрон в Offboard режим:
   ```python
   self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
   ```
   После этого дрон успешно входит в **Offboard** (NavState = 14), где я могу управлять дроном.

3. Пытаюсь выйти из Offboard обратно в Loiter:
   ```python
   self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=5.0)
   ```
   Однако NavState остаётся равным **14 (Offboard)**, и дрон продолжает находиться в этом режиме.

#### **Ожидаемое поведение**
После отправки команды на переключение в **Loiter**, дрон должен перейти в NavState = 5, и начать зависание в текущей позиции.

#### **Реальное поведение**
NavState остаётся равным **14 (Offboard)**, и команда переключения не влияет на режим. 


#### **Вопрос**
Каким образом можно гарантированно выйти из режима **Offboard** и переключиться в **Loiter**? Есть ли дополнительные условия или настройки в PX4, которые необходимо учесть?

Буду благодарен за любые рекомендации или разъяснения!


### **Topic Title**
How to properly exit Offboard mode and switch to Loiter (NavState = 5)?

---

I am facing an issue when trying to exit **Offboard** mode and switch the drone to **Loiter (NavState = 5)**. I am following the example and guidelines from the [px4_ros_com repository](https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard_py/offboard_control.py).

#### **My Configuration**
- **Language:** Python
- **Platform:** PX4
- **Firmware version:** v1.13

#### **What I Am Doing**
1. I execute a takeoff command:
   ```python
   self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5.0)
   ```
   After this, the drone switches to a hover mode (I assume this is **Loiter**).

2. I switch the drone to Offboard mode:
   ```python
   self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
   ```
   After this, the drone successfully enters **Offboard (NavState = 14)**, and I can control it.

3. I attempt to switch back from Offboard to Loiter:
   ```python
   self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=5.0)
   ```
   However, the `NavState` remains **14 (Offboard)**, and the drone continues in this mode.

#### **Expected Behavior**
After sending the command to switch to **Loiter**, the drone should transition to `NavState = 5` and begin hovering at its current position.

#### **Actual Behavior**
The `NavState` remains **14 (Offboard)**, and the switch command has no effect on the mode.

---

#### **Question**
How can I reliably exit **Offboard** mode and switch to **Loiter**? Are there additional conditions or settings in PX4 that need to be considered?

I would greatly appreciate any recommendations or explanations! 
