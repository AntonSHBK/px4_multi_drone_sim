## **Сборка и запуск проекта**

### **Сборка проекта**

```bash
colcon build --packages-select multi_drone multi_drone_msg
```

**Описание:**
- Собираются указанные пакеты `multi_drone` и `multi_drone_msg`.
- Перед выполнением команды убедитесь, что вы находитесь в корневой директории вашего ROS 2 рабочего пространства.

---

### **Запуск**

#### **1. Запуск тестовой среды Gazebo**
```bash
ros2 launch multi_drone test_gazebo.launch.py
```
**Описание:**
- Запускает тестовую симуляцию Gazebo для взаимодействия с дроном.

#### **2. Запуск нескольких дронов**
```bash
ros2 launch multi_drone test_multi_drone_run.launch.py
```
**Описание:**
- Запускает симуляцию с несколькими дронами.

#### **3. Запуск стандартного контроллера**
```bash
ros2 launch multi_drone test_run.launch.py
```
**Описание:**
- Запускает стандартный сценарий управления дроном.

---

## **Работа с топиками**

#### **Просмотр топика с информацией о дроне**
```bash
ros2 topic echo /id_1_x500/out/inform_of_drone
```
**Описание:**
- Выводит сообщения из топика `/id_1_x500/out/inform_of_drone`.
- Замените `id_1_x500` на идентификатор вашего дрона, если он отличается.

---

### Полезные команды ROS 2:

- **Проверка активных нод:**
  ```bash
  ros2 node list
  ```

- **Просмотр всех доступных топиков:**
  ```bash
  ros2 topic list
  ```

- **Информация о топике:**
  ```bash
  ros2 topic info /topic_name
  ```

- **Публикация тестового сообщения:**
  ```bash
  ros2 topic pub /topic_name std_msgs/String "data: 'Hello, Drone!'"
  ```

- **Запуск RQT для визуализации данных:**
  ```bash
  rqt
  ```

---

## **Отправка G-команд**

### **Базовые команды**

#### **Пример отправки команд G0-G6**
```bash
#!/bin/bash

# G0: Сброс команд
ros2 topic pub --once /id_1_x500/in/command_json std_msgs/msg/String "{data: '{\"name\": \"G0\", \"counter\": 0}'}"

# G1: Армирование
ros2 topic pub --once /id_1_x500/in/command_json std_msgs/msg/String "{data: '{\"name\": \"G1\", \"counter\": 1}'}"

# G2: Разармирование
ros2 topic pub --once /id_1_x500/in/command_json std_msgs/msg/String "{data: '{\"name\": \"G2\", \"counter\": 2}'}"

# G3: Взлёт на высоту 2.0 м
ros2 topic pub --once /id_1_x500/in/command_json std_msgs/msg/String "{data: '{\"name\": \"G3\", \"counter\": 3, \"altitude\": 2.0}'}"

# G4: Посадка
ros2 topic pub --once /id_1_x500/in/command_json std_msgs/msg/String "{data: '{\"name\": \"G4\", \"counter\": 4}'}"

# G5: Удержание позиции
ros2 topic pub --once /id_1_x500/in/command_json std_msgs/msg/String "{data: '{\"name\": \"G5\", \"counter\": 5}'}"

# G6: Включение режима Offboard
ros2 topic pub --once /id_1_x500/in/command_json std_msgs/msg/String "{data: '{\"name\": \"G6\", \"counter\": 6}'}"

echo "Базовые команды G0-G6 отправлены."
```

---

### **Сложные команды**

#### **1. Команда G20: Перемещение в точку**
```bash
#!/bin/bash

TOPIC="/id_1_x500/in/command_json"

ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G20\",
    \"counter\": 1,
    \"x\": 0.0,
    \"y\": 0.0,
    \"z\": 6.0,
    \"yaw\": 1.57,
    \"velocity\": 2.0,
    \"coordinate_system\": \"global_ENU\"
}'}"

echo "Команда G20_MoveToPoint отправлена."
```

---

#### **2. Команда G21: Линейное перемещение**
```bash
#!/bin/bash

TOPIC="/id_1_x500/in/command_json"

ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G21\",
    \"counter\": 2,
    \"start_point\": [0.0, 0.0, 20.0],
    \"end_point\": [5.0, 10.0, 25.0],
    \"velocity\": 1.5,
    \"yaw\": 1.57,
    \"coordinate_system\": \"global_ENU\"
}'}"

echo "Команда G21_LinearMove отправлена."
```

---

#### **3. Команда G22: Круговая траектория**
```bash
#!/bin/bash

TOPIC="/id_1_x500/in/command_json"

ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G22\",
    \"counter\": 3,
    \"start_point\": [5.0, 5.0, 20.0],
    \"end_point\": [10.0, 10.0, 20.0],
    \"radius\": 10.0,
    \"direction\": \"CCW\",
    \"points_count\": 10,
    \"coordinate_system\": \"global_ENU\"
}'}"

echo "Команда G22_CircularTrajectory отправлена."
```

---

#### **4. Команда G23: Полёт по орбите**
```bash
#!/bin/bash

TOPIC="/id_1_x500/in/command_json"

ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G23\",
    \"counter\": 4,
    \"center_point\": [0.0, 0.0, 0.0],
    \"radius\": 5.0,
    \"angular_velocity\": 0.1,
    \"orbit_direction\": \"CW\",
    \"yaw_mode\": \"facing_center\",
    \"duration\": 30,
    \"coordinate_system\": \"global_ENU\"
}'}"

echo "Команда G23_Orbit отправлена."
```

---

#### **5. Команда G24: Спиральная траектория**
```bash
#!/bin/bash

TOPIC="/id_1_x500/in/command_json"

ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G24\",
    \"counter\": 5,
    \"center_point\": [0.0, 0.0, 0.0],
    \"radius_start\": 3.0,
    \"radius_end\": 8.0,
    \"height_change\": 10.0,
    \"turns\": 3,
    \"direction\": \"CW\",
    \"points_per_turn\": 10,
    \"coordinate_system\": \"global_ENU\"
}'}"

echo "Команда G24_SpiralTrajectory отправлена."
```