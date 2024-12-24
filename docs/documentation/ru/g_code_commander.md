### g_code_commander.md

#### **Описание `X500Commander`**

Класс `X500Commander` реализует обработку и выполнение высокоуровневых команд в формате G-кодов для дрона X500. Он расширяет функциональность базового класса `DroneCommander`, предоставляя инструменты для управления очередями команд, их последовательного выполнения и обработки специальных сценариев.

---

### **1. Базовый класс `DroneCommander`**

`DroneCommander` — это абстрактный класс, который задаёт структуру для создания командеров. Основные возможности:
- Управление очередью команд.
- Обработка входящих JSON-команд.
- Мониторинг состояния активной команды.

---

### **2. Основные функции `X500Commander`**

#### **2.1 Инициализация**

Конструктор класса инициализирует следующие компоненты:
- **Регистрация всех команд:** Используя модуль `g_code_module`, регистрируются все классы, наследующие `BaseGCommand`. Это позволяет динамически добавлять новые команды.
- **Подписка на входящие команды:** Через ROS подписку на топик `<{prefix_name}>/in/command_json` происходит получение JSON-команд.
- **Таймер выполнения команд:** С помощью ROS-таймера команды из очереди обрабатываются с заданной периодичностью.

```python
def __init__(self, controller: "X500Controller", timer_execution=0.1)
```

---

#### **2.2 Обработка JSON-команд**

1. **Приём JSON-команды:**
   Метод `command_json_callback` обрабатывает входящие ROS-сообщения, декодируя JSON и передавая данные в метод `process_incoming_command`.
   ```python
   def command_json_callback(self, msg: String)
   ```

2. **Обработка входящих данных:**
   Метод `process_incoming_command` извлекает имя команды (`name`) и находит соответствующий класс команды из зарегистрированных.
   ```python
   def process_incoming_command(self, data: dict)
   ```

3. **Создание команды:**
   Используется статический метод `from_dict` класса команды для преобразования JSON в экземпляр команды.
   ```python
   command_class.from_dict(data)
   ```

4. **Добавление в очередь:**
   Если команда не является специальной (например, `G0`), она добавляется в очередь.

---

#### **2.3 Выполнение команд**

1. **Таймер выполнения:**
   Метод `_timer_execute_commands_callback` вызывается ROS-таймером и проверяет состояние активной команды или берёт следующую из очереди.
   ```python
   def _timer_execute_commands_callback(self)
   ```

2. **Проверка завершения:**
   Если команда завершена (`is_complete`), вызывается метод `handle_completion`, и активная команда очищается.

3. **Выполнение следующей команды:**
   Если очередь команд не пуста, активируется следующая команда, вызывая её метод `safe_execute`.

---

#### **2.4 Специальные команды**

Метод `check_special_command` проверяет, является ли команда специальной (например, `G0`) и обрабатывает её сразу, не добавляя в очередь.

```python
def check_special_command(self, command: BaseGCommand) -> bool
```

---

### Как составить команду движения для дрона

#### **Общий принцип**

Команда движения дрона формируется в виде класса, унаследованного от базового класса `BaseMoveGCommand`. Она описывает:
1. **Тип движения** (линейное перемещение, круговая траектория и т.д.).
2. **Целевые точки** (начальная и конечная позиции).
3. **Параметры движения** (скорость, ориентация `yaw`).

Каждая команда должна реализовывать ключевые методы:
- `update_targets_positions`: формирует список целевых точек.
- `execute`: отправляет команды на дрон для выполнения движения.
- `is_complete`: проверяет завершение текущего этапа или всей команды.

---

### **Шаги создания команды**

#### **1. Унаследуйте класс от `BaseMoveGCommand`**

Базовый класс предоставляет базовую структуру для реализации команд движения:
- Хранит список целевых точек (`targets_positions`).
- Определяет общую логику выполнения и проверки завершения.

#### **2. Определите параметры команды**

Пример: Для линейного перемещения (`G21_LinearMove`) необходимо задать начальную и конечную точки, скорость и угол ориентации `yaw`.

```python
class G21_LinearMove(BaseMoveGCommand):
    """
    Команда для линейного перемещения дрона между двумя точками.
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

#### **3. Реализуйте метод `update_targets_positions`**

Этот метод формирует список целевых позиций. Например, для линейного движения это начальная и конечная точки.

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

#### **4. Реализуйте методы сериализации и десериализации**

Эти методы позволяют сохранять и загружать параметры команды из словаря, например, для передачи через JSON.

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

### **Пример использования команды**

1. **Создание команды:**
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

2. **Передача команды через JSON:**
   ```python
   import json

   json_data = json.dumps(command.to_dict())
   print(json_data)
   ```

3. **Загрузка команды из JSON:**
   ```python
   loaded_command = G21_LinearMove.from_dict(json.loads(json_data))
   print(loaded_command)
   ```

4. **Добавление команды в `X500Commander`:**
   ```python
   commander.process_incoming_command(command.to_dict())
   ```

---

### **Пример JSON-команды**

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

### **Основные параметры**

1. **`name` (str):** Имя команды (например, `G21`).
2. **`start_point` (list):** Начальная точка `[x, y, z]`.
3. **`end_point` (list):** Конечная точка `[x, y, z]`.
4. **`velocity` (float):** Скорость движения (в м/с).
5. **`yaw` (float):** Угол ориентации (в радианах).
6. **`coordinate_system` (str):** Система координат (`local_NED`, `local_ENU`, `global_ENU`, `global_NED`).



