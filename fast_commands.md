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
