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