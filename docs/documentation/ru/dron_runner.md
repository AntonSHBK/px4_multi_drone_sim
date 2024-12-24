### dron_runner.md

#### **Описание функционала `runner.py`**

Файл `runner.py` предоставляет интерфейс для автоматического запуска симуляции дрона с использованием PX4 и ROS 2. Основной функционал включает:
- Создание процессов для автопилота PX4.
- Создание ROS 2 нод для управления дронами.
- Настройку запуска для одного или нескольких дронов.

---

### **Основные классы**

#### **1. PX4Process**
Этот класс отвечает за создание и настройку процесса PX4, который симулирует работу автопилота для дрона.

##### **Аргументы конструктора:**
- `drone_id` (int): Уникальный идентификатор дрона.
- `drone_type` (str): Тип дрона (например, `'x500'`).
- `spawn_position` (list): Стартовая позиция и ориентация дрона в формате `[x, y, z, roll, pitch, yaw]`.
- `px4_autostart` (int): Код автозапуска PX4 для конфигурации дрона.
- `px4_dir` (str): Путь к директории PX4 Autopilot.
- `terminal` (str): Терминал для запуска процесса (`'gnome-terminal'`, `'xterm'`, `'konsole'`, `'bash'`).

##### **Методы:**
- `create_process() -> ExecuteProcess`: Создает процесс PX4 для запуска симуляции.

##### **Пример использования:**
```python
px4_process = PX4Process(
    drone_id=1,
    drone_type='x500',
    spawn_position=[0, 0, 0, 0, 0, 0],
    px4_autostart=4001,
    px4_dir="/workspace/src/PX4-Autopilot/",
    terminal='gnome-terminal'
)
px4_execute_process = px4_process.create_process()
```

---

#### **2. ControllerNode**
Этот класс используется для создания ROS 2 ноды, управляющей поведением дрона.

##### **Аргументы конструктора:**
- `package_name` (str): Имя ROS 2 пакета, содержащего контроллер.
- `controller_script` (str): Имя исполняемого файла ноды.
- `drone_id` (int): Уникальный идентификатор дрона.
- `drone_type` (str): Тип дрона (например, `'x500'`).
- `spawn_position` (list): Стартовая позиция и ориентация дрона в формате `[x, y, z, roll, pitch, yaw]`.
- `output` (str): Тип вывода (`'log'` или `'screen'`).
- `additional_params` (dict): Дополнительные параметры для ROS 2 ноды.

##### **Методы:**
- `create_node() -> Node`: Создает ROS 2 ноду с заданными параметрами.

##### **Пример использования:**
```python
controller_node = ControllerNode(
    package_name='multi_drone',
    controller_script='x500.py',
    drone_id=1,
    drone_type='x500',
    spawn_position=[0, 0, 0, 0, 0, 0],
    output='log'
)
ros2_node = controller_node.create_node()
```

---

#### **3. Функция `launch_robot`**
Эта функция объединяет PX4 и ROS 2 компоненты в единый список процессов и нод для запуска симуляции одного дрона.

##### **Аргументы:**
- `drone_id` (int): Идентификатор дрона.
- `drone_type` (str): Тип дрона.
- `spawn_position` (list): Позиция и ориентация дрона.
- `px4_autostart` (int): Конфигурация PX4 автозапуска.
- `px4_dir` (str): Путь к PX4 Autopilot.
- `terminal` (str): Терминал для запуска PX4.
- `package_name` (str): Имя ROS 2 пакета.
- `controller_script` (str): Имя скрипта контроллера.
- `additional_params` (dict): Дополнительные параметры для ROS 2 ноды.

##### **Пример использования:**
```python
nodes = launch_robot(
    drone_id=1,
    drone_type='x500',
    spawn_position=[0, 0, 0, 0, 0, 0],
    px4_autostart=4001,
    px4_dir="/workspace/src/PX4-Autopilot/",
    terminal='gnome-terminal',
    package_name='multi_drone',
    controller_script='x500.py',
    additional_params={}
)
```

---

### **Ключевые функции**

#### **1. `get_microxrce_agent_exec`**
Создает процесс для запуска `MicroXRCEAgent`, который обеспечивает взаимодействие между PX4 и ROS 2.

##### **Аргументы:**
- `udp` (str): Протокол (по умолчанию `'udp4'`).
- `port` (str): Порт (по умолчанию `'8888'`).

##### **Пример использования:**
```python
microxrce_agent = get_microxrce_agent_exec(udp='udp4', port='8888')
```

---

### **Пример полного запуска**

```python
from runner import launch_robot

# Запуск одного дрона с ID=1
nodes = launch_robot(
    drone_id=1,
    drone_type='x500',
    spawn_position=[0, 0, 0, 0, 0, 0],
    px4_autostart=4001,
    px4_dir="/workspace/src/PX4-Autopilot/",
    terminal='gnome-terminal',
    package_name='multi_drone',
    controller_script='x500.py',
    additional_params={}
)

for node in nodes:
    print(f"Запуск процесса: {node}")
```
