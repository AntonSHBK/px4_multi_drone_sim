### G_code_command.md

#### **Как отправлять G-коды на соответствующие дроны**

Команды G-кодов используются для управления действиями дронов. Эти команды передаются в формате JSON через топики ROS 2 и интерпретируются командерами дронов. В данном руководстве описаны шаги по отправке стандартных и пользовательских G-команд.

---

### **Основной принцип**

1. **G-коды** — это JSON-команды, которые содержат:
   - Имя команды (`name`).
   - Счётчик команды (`counter`).
   - Параметры команды, зависящие от её типа (например, позиция, скорость).
2. **Топики для команд:**
   - Каждая команда отправляется в соответствующий топик дрона, например: `/id_1_x500/in/command_json`.
3. **Используемый формат:**
   - Команды должны быть в формате JSON.

---

### **Примеры отправки G-команд**

### **Основные команды**

#### **1. G0: Сброс команд**
Команда `G0` останавливает выполнение текущей команды, очищает очередь команд и возвращает дрон в безопасное состояние.

**Пример JSON:**
```json
{
    "name": "G0",
    "counter": 0
}
```

**Описание:**
- Полностью останавливает активные команды.
- Очищает очередь команд.

**Пример отправки:**
```bash
ros2 topic pub --once /id_1_x500/in/command_json std_msgs/msg/String "{data: '{\"name\": \"G0\", \"counter\": 0}'}"
```



#### **2. Отправка базовых команд**

Эти команды управляют базовыми действиями дрона: армирование, взлёт, посадка и т.д.

##### **G-коды: G1-G6**
```bash
#!/bin/bash

# Топик для отправки команд
TOPIC="/id_1_x500/in/command_json"

# G1: Армирование
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G1\",
    \"counter\": 1
}'}"

# G2: Разармирование
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G2\",
    \"counter\": 2
}'}"

# G3: Взлёт на высоту 2.0 м
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G3\",
    \"counter\": 3,
    \"altitude\": 2.0
}'}"

# G4: Посадка
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G4\",
    \"counter\": 4
}'}"

# G5: Удержание позиции
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G5\",
    \"counter\": 5
}'}"

# G6: Включение Offboard режима
ros2 topic pub --once $TOPIC std_msgs/msg/String "{data: '{
    \"name\": \"G6\",
    \"counter\": 6
}'}"

echo "Базовые команды отправлены."
```

#### **Общие рекомендации**

1. **Проверка топиков:**
   - Перед отправкой команд убедитесь, что топик дрона активен:
     ```bash
     ros2 topic list
     ```

2. **JSON-валидатор:**
   - Используйте онлайн-валидаторы JSON для проверки синтаксиса команд.

3. **Указание системы координат:**
   - `coordinate_system` может быть одним из:
     - `global_ENU`
     - `local_NED`
     - `local_ENU`
     - `global_NED`

---

#### **1. G20: Перемещение в точку**
Команда `G20_MoveToPoint` используется для перемещения дрона к указанной точке с возможностью указания угла ориентации (`yaw`) и скорости.

**Пример JSON:**
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

**Описание параметров:**
- `x`, `y`, `z`: Координаты целевой точки.
- `yaw`: Угол ориентации (в радианах).
- `velocity`: Скорость перемещения (в м/с).
- `coordinate_system`: Система координат (`global_ENU`, `local_ENU`, и т.д.).

**Пример отправки:**
```bash
#!/bin/bash

# Топик для отправки команды
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

echo "Команда G20_MoveToPoint отправлена."
```

---

#### **2. G21: Линейное перемещение**
Команда `G21_LinearMove` задаёт линейное движение между двумя точками.

**Пример JSON:**
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

**Описание параметров:**
- `start_point`: Начальная точка `[x, y, z]`.
- `end_point`: Конечная точка `[x, y, z]`.
- `velocity`: Скорость движения (в м/с).
- `yaw`: Угол ориентации (в радианах).
- `coordinate_system`: Система координат.

**Пример отправки:**
```bash
#!/bin/bash

# Топик для отправки команды
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

echo "Команда G21_LinearMove отправлена."
```

---

#### **3. G22: Круговая траектория**
Команда `G22_CircularTrajectory` задаёт перемещение по дуге.

**Пример JSON:**
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

**Описание параметров:**
- `start_point`: Начальная точка `[x, y, z]`.
- `end_point`: Конечная точка `[x, y, z]`.
- `radius`: Радиус траектории (в м).
- `direction`: Направление (`CW` — по часовой, `CCW` — против часовой).
- `points_count`: Количество точек на дуге.
- `coordinate_system`: Система координат.

**Пример отправки:**
```bash
#!/bin/bash

# Топик для отправки команд
TOPIC="/id_1_x500/in/command_json"

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
```

---

#### *4. G23: Полёт по орбите**
Команда `G23_Orbit` задаёт полёт по орбите вокруг заданной точки.

**Пример JSON:**
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

**Описание параметров:**
- `center_point`: Центр орбиты `[x, y, z]`.
- `radius`: Радиус орбиты (в м).
- `angular_velocity`: Угловая скорость (в радианах/с).
- `orbit_direction`: Направление (`CW`, `CCW`).
- `yaw_mode`: Режим ориентации (`fixed`, `facing_center`).
- `duration`: Продолжительность выполнения (в секундах).

**Пример отправки:**
```bash
#!/bin/bash

# Топик для отправки команд
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