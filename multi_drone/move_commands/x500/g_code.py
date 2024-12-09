# g_code.py

import numpy as np

from multi_drone.move_commands.base.base_g_code import BaseGCommand


class G20_MoveToPoint(BaseGCommand):
    """
    Команда для перемещения дрона к одной заданной точке с возможностью указания угла ориентации (yaw) и скорости.
    """
    def __init__(self, 
                 counter: int, 
                 x: float = float('nan'), 
                 y: float = float('nan'), 
                 z: float = float('nan'), 
                 yaw: float = float('nan'),
                 velocity: float = float('nan')):
        super().__init__("G20", counter)
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.velocity = velocity

    def execute(self, controller):
        """
        Отправляет команду перемещения к указанной точке через контроллер.
        """
        if not self.complete:
            target_position = [self.x, self.y, self.z]
            controller.move_to_point(
                position=target_position,
                yaw=self.yaw,
                velocity=self.velocity
            )

    def is_complete(self, controller) -> bool:
        """
        Проверяет, достиг ли дрон целевой точки.
        """
        current_position = controller.get_current_position()
        target_position = np.array([self.x, self.y, self.z])
        yaw_tolerance = 0.1  # Радианы
        position_tolerance = 0.1  # Метры

        # Проверка позиции
        position_error = np.linalg.norm(current_position - target_position)
        if position_error > position_tolerance:
            return False

        # Проверка yaw, если он задан
        if not np.isnan(self.yaw):
            current_yaw = controller.get_current_yaw()
            yaw_error = abs(current_yaw - self.yaw)
            if yaw_error > yaw_tolerance:
                return False

        self.complete = True
        return True

    def to_dict(self) -> dict:
        """
        Сериализует команду в словарь.
        """
        return {
            **super().to_dict(),
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw": self.yaw,
            "velocity": self.velocity
        }

    @classmethod
    def from_dict(cls, data: dict):
        """
        Десериализует команду из словаря.
        """
        return cls(
            counter=data["counter"],
            x=data.get("x", float('nan')),
            y=data.get("y", float('nan')),
            z=data.get("z", float('nan')),
            yaw=data.get("yaw", float('nan')),
            velocity=data.get("velocity", float('nan'))
        )

    def __repr__(self):
        """
        Строковое представление команды.
        """
        return (f"G20_MoveToPoint(counter={self.counter}, x={self.x}, y={self.y}, "
                f"z={self.z}, yaw={self.yaw}, velocity={self.velocity}, complete={self.complete})")



class G21_LinearMove(BaseGCommand):
    """
    Команда для линейного перемещения между двумя точками.
    """
    def __init__(self, 
                 counter, 
                 x1, y1, z1, 
                 x2, y2, z2, 
                 velocity=None, 
                 yaw=None, 
                 num_segments=2, 
                 default_velocity=float('nan'), 
                 default_yaw=float('nan'), 
                 current_step=0):
        super().__init__("G21", counter)
        self.x1 = x1
        self.y1 = y1
        self.z1 = z1
        self.x2 = x2
        self.y2 = y2
        self.z2 = z2
        self.num_segments = num_segments
        self.default_velocity = default_velocity
        self.default_yaw = default_yaw
        self.current_step = current_step

        # Убедимся, что длина velocity и yaw соответствует num_segments
        self.velocity = self._adjust_list_length(velocity, num_segments, default_value=self.default_velocity)
        self.yaw = self._adjust_list_length(yaw, num_segments, default_value=self.default_yaw)

        # Инициализируем список точек траектории
        self.init_trajectory_setpoint_list()

    def to_dict(self):
        return {
            "name": self.name,
            "counter": self.counter_command,
            "x1": self.x1,
            "y1": self.y1,
            "z1": self.z1,
            "x2": self.x2,
            "y2": self.y2,
            "z2": self.z2,
            "velocity": self.velocity,
            "yaw": self.yaw,
            "num_segments": self.num_segments,
            "current_step": self.current_step
        }

    def from_dict(self, data):
        self.counter_command = data.get("counter", self.counter_command)
        self.x1 = data.get("x1", self.x1)
        self.y1 = data.get("y1", self.y1)
        self.z1 = data.get("z1", self.z1)
        self.x2 = data.get("x2", self.x2)
        self.y2 = data.get("y2", self.y2)
        self.z2 = data.get("z2", self.z2)
        self.num_segments = data.get("num_segments", self.num_segments)
        self.current_step = data.get("current_step", self.current_step)

        # Убедимся, что длина velocity и yaw соответствует num_segments
        self.velocity = self._adjust_list_length(data.get("velocity", []), self.num_segments, default_value=self.default_velocity)
        self.yaw = self._adjust_list_length(data.get("yaw", []), self.num_segments, default_value=self.default_yaw)

        # Переинициализация списка с обновленными данными
        self.init_trajectory_setpoint_list()

    def get_trajectory_setpoint(self) -> TrajectorySetpoint:
        """
        Возвращает текущий `TrajectorySetpoint` на основе текущего шага `current_step`.
        """
        if self.current_step < len(self.trajectory_setpoint_list):
            return self.trajectory_setpoint_list[self.current_step]
        else:
            self.complete = True
            self.current_step = 0
            return None

    def init_trajectory_setpoint_list(self):
        """
        Инициализация списка точек траектории с объектами TrajectorySetpoint.
        """
        self.trajectory_setpoint_list = []
        
        for i in range(self.num_segments + 1):
            t = i / self.num_segments
            trajectory = TrajectorySetpoint()

            # Линейная интерполяция координат
            trajectory.position[0] = self.x1 + t * (self.x2 - self.x1)
            trajectory.position[1] = self.y1 + t * (self.y2 - self.y1)
            trajectory.position[2] = self.z1 + t * (self.z2 - self.z1)

            # Устанавливаем скорость: используем значение из списка или default_velocity
            trajectory.velocity[0] = self.velocity[i] if i < len(self.velocity) else self.default_velocity
            trajectory.velocity[1] = float('nan')  # Оставляем NaN для Y и Z
            trajectory.velocity[2] = float('nan')

            # Устанавливаем yaw: используем значение из списка или default_yaw
            trajectory.yaw = self.yaw[i] if i < len(self.yaw) else self.default_yaw

            # Добавляем точку в список траекторий
            self.trajectory_setpoint_list.append(trajectory)

    def _adjust_list_length(self, input_list, target_length, default_value):
        """
        Приводит длину списка к target_length.
        Если список короче, добавляет default_value.
        Если длиннее, обрезает до target_length.
        """
        if not input_list:
            input_list = []
        if len(input_list) < target_length:
            input_list.extend([default_value] * (target_length - len(input_list)))
        elif len(input_list) > target_length:
            input_list = input_list[:target_length]
        return input_list


class G22_CircularMove(BaseGCommand):
    """
    Команда для движения по окружности или дуге с заданным радиусом, начальным и конечным углом.

    Аргументы конструктора:
        counter (int): Порядковый номер команды.
        center_x, center_y, center_z (float): Координаты центра окружности.
        radius (float): Радиус окружности или дуги.
        start_angle (float): Начальный угол в радианах.
        end_angle (float): Конечный угол в радианах.
        clockwise (bool): Направление движения:
            True - по часовой стрелке.
            False - против часовой стрелки.
        num_segments (int): Количество сегментов, на которые разбивается дуга (по умолчанию 10).
        velocity (list[float] | None): Список скоростей для каждого сегмента (по умолчанию `None`).
        yaw (list[float] | None): Список углов ориентации (yaw) для каждого сегмента (по умолчанию `None`).
        default_velocity (float): Скорость по умолчанию, если не указаны скорости (по умолчанию `float('nan')`).
        default_yaw (float): Угол yaw по умолчанию, если не указаны yaw (по умолчанию `float('nan')`).
        current_step (int): Текущий шаг выполнения команды (по умолчанию 0).

    Переменные:
        trajectory_setpoint_list (list[TrajectorySetpoint]): Список точек траектории.
        velocity (list[float]): Список скоростей, соответствующих сегментам.
        yaw (list[float]): Список углов yaw, соответствующих сегментам.

    Подход:
        1. Распределяются углы между начальным и конечным углом в соответствии с `num_segments`.
        2. Если направление `clockwise=True`, углы корректируются для движения по часовой стрелке.
        Если `clockwise=False`, углы корректируются для движения против часовой стрелки.
        3. Для каждого угла рассчитываются координаты точки на окружности, скорость и yaw.
        4. Все точки траектории сохраняются в `trajectory_setpoint_list`.
    """
    def __init__(self, 
                 counter, 
                 center_x, center_y, center_z, 
                 radius, 
                 start_angle, 
                 end_angle, 
                 clockwise=True, 
                 num_segments=10, 
                 velocity=None, 
                 yaw=None, 
                 default_velocity=float('nan'), 
                 default_yaw=float('nan'), 
                 current_step=0):
        super().__init__("G22", counter)
        self.center_x = center_x
        self.center_y = center_y
        self.center_z = center_z
        self.radius = radius
        self.start_angle = start_angle  # В радианах
        self.end_angle = end_angle  # В радианах
        self.clockwise = clockwise
        self.num_segments = num_segments
        self.default_velocity = default_velocity
        self.default_yaw = default_yaw
        self.current_step = current_step

        # Убедимся, что длина velocity и yaw соответствует num_segments
        self.velocity = self._adjust_list_length(velocity, num_segments, default_value=self.default_velocity)
        self.yaw = self._adjust_list_length(yaw, num_segments, default_value=self.default_yaw)

        # Инициализируем список точек траектории
        self.init_trajectory_setpoint_list()

    def to_dict(self):
        return {
            "name": self.name,
            "counter": self.counter_command,
            "center_x": self.center_x,
            "center_y": self.center_y,
            "center_z": self.center_z,
            "radius": self.radius,
            "start_angle": self.start_angle,
            "end_angle": self.end_angle,
            "clockwise": self.clockwise,
            "velocity": self.velocity,
            "yaw": self.yaw,
            "num_segments": self.num_segments,
            "current_step": self.current_step
        }

    def from_dict(self, data):
        self.counter_command = data.get("counter", self.counter_command)
        self.center_x = data.get("center_x", self.center_x)
        self.center_y = data.get("center_y", self.center_y)
        self.center_z = data.get("center_z", self.center_z)
        self.radius = data.get("radius", self.radius)
        self.start_angle = data.get("start_angle", self.start_angle)
        self.end_angle = data.get("end_angle", self.end_angle)
        self.clockwise = data.get("clockwise", self.clockwise)
        self.num_segments = data.get("num_segments", self.num_segments)
        self.current_step = data.get("current_step", self.current_step)

        # Убедимся, что длина velocity и yaw соответствует num_segments
        self.velocity = self._adjust_list_length(data.get("velocity", []), self.num_segments, default_value=self.default_velocity)
        self.yaw = self._adjust_list_length(data.get("yaw", []), self.num_segments, default_value=self.default_yaw)

        # Переинициализация списка с обновленными данными
        self.init_trajectory_setpoint_list()

    def get_trajectory_setpoint(self) -> TrajectorySetpoint:
        """
        Возвращает текущий `TrajectorySetpoint` на основе текущего шага `current_step`.
        """
        if self.current_step < len(self.trajectory_setpoint_list):
            return self.trajectory_setpoint_list[self.current_step]
        else:
            self.complete = True
            self.current_step = 0
            return None

    def init_trajectory_setpoint_list(self):
        """
        Инициализация списка точек траектории с объектами TrajectorySetpoint.
        """
        self.trajectory_setpoint_list = []

        # Расчет углов дуги с учетом направления движения
        if self.clockwise:
            if self.end_angle > self.start_angle:
                self.end_angle -= 2 * np.pi
        else:
            if self.end_angle < self.start_angle:
                self.end_angle += 2 * np.pi

        # Линейно распределяем углы между началом и концом
        angles = np.linspace(self.start_angle, self.end_angle, self.num_segments + 1)

        # Генерация точек траектории
        for i, angle in enumerate(angles):
            trajectory = TrajectorySetpoint()

            # Рассчитываем координаты точки на окружности
            trajectory.position[0] = self.center_x + self.radius * np.cos(angle)
            trajectory.position[1] = self.center_y + self.radius * np.sin(angle)
            trajectory.position[2] = self.center_z

            # Устанавливаем скорость: используем значение из списка или default_velocity
            trajectory.velocity[0] = self.velocity[i] if i < len(self.velocity) else self.default_velocity
            trajectory.velocity[1] = float('nan')  # Оставляем NaN для Y и Z
            trajectory.velocity[2] = float('nan')

            # Устанавливаем yaw: используем значение из списка или default_yaw
            trajectory.yaw = self.yaw[i] if i < len(self.yaw) else self.default_yaw

            # Добавляем точку в список траекторий
            self.trajectory_setpoint_list.append(trajectory)

    def _adjust_list_length(self, input_list, target_length, default_value):
        """
        Приводит длину списка к target_length.
        Если список короче, добавляет default_value.
        Если длиннее, обрезает до target_length.
        """
        if not input_list:
            input_list = []
        if len(input_list) < target_length:
            input_list.extend([default_value] * (target_length - len(input_list)))
        elif len(input_list) > target_length:
            input_list = input_list[:target_length]
        return input_list
    
    
class G23_ArcMove(BaseGCommand):
    """
    Команда для перемещения по дуге между двумя точками с заданным радиусом.

    Аргументы конструктора:
        counter (int): Порядковый номер команды.
        start_x, start_y, start_z (float): Начальные координаты дуги.
        end_x, end_y, end_z (float): Конечные координаты дуги.
        radius (float): Радиус дуги.
        clockwise (bool): Направление движения:
            True - по часовой стрелке.
            False - против часовой стрелки.
        num_segments (int): Количество сегментов, на которые разбивается дуга (по умолчанию 10).
        velocity (list[float] | None): Список скоростей для каждого сегмента (по умолчанию `None`).
        yaw (list[float] | None): Список углов ориентации (yaw) для каждого сегмента (по умолчанию `None`).
        default_velocity (float): Скорость по умолчанию, если не указаны скорости (по умолчанию `float('nan')`).
        default_yaw (float): Угол yaw по умолчанию, если не указаны yaw (по умолчанию `float('nan')`).
        current_step (int): Текущий шаг выполнения команды (по умолчанию 0).

    Переменные:
        trajectory_setpoint_list (list[TrajectorySetpoint]): Список точек траектории.
        velocity (list[float]): Список скоростей, соответствующих сегментам.
        yaw (list[float]): Список углов yaw, соответствующих сегментам.

    Подход:
        1. Вычисляется центр дуги на основе начальной и конечной точек, радиуса и направления.
        2. Распределяются углы между начальной и конечной точкой в соответствии с `num_segments`.
        3. Для каждого угла рассчитываются координаты точки на дуге, скорость и yaw.
        4. Все точки траектории сохраняются в `trajectory_setpoint_list`.
    """
    def __init__(self, 
                 counter, 
                 start_x, start_y, start_z, 
                 end_x, end_y, end_z, 
                 radius, 
                 clockwise=True, 
                 num_segments=10, 
                 velocity=None, 
                 yaw=None, 
                 default_velocity=float('nan'), 
                 default_yaw=float('nan'), 
                 current_step=0):
        super().__init__("G23", counter)
        self.start_x = start_x
        self.start_y = start_y
        self.start_z = start_z
        self.end_x = end_x
        self.end_y = end_y
        self.end_z = end_z
        self.radius = radius
        self.clockwise = clockwise
        self.num_segments = num_segments
        self.default_velocity = default_velocity
        self.default_yaw = default_yaw
        self.current_step = current_step

        # Убедимся, что длина velocity и yaw соответствует num_segments
        self.velocity = self._adjust_list_length(velocity, num_segments, default_value=self.default_velocity)
        self.yaw = self._adjust_list_length(yaw, num_segments, default_value=self.default_yaw)

        # Инициализируем список точек траектории
        self.init_trajectory_setpoint_list()

    def to_dict(self):
        return {
            "name": self.name,
            "counter": self.counter_command,
            "start_x": self.start_x,
            "start_y": self.start_y,
            "start_z": self.start_z,
            "end_x": self.end_x,
            "end_y": self.end_y,
            "end_z": self.end_z,
            "radius": self.radius,
            "clockwise": self.clockwise,
            "velocity": self.velocity,
            "yaw": self.yaw,
            "num_segments": self.num_segments,
            "current_step": self.current_step
        }

    def from_dict(self, data):
        self.counter_command = data.get("counter", self.counter_command)
        self.start_x = data.get("start_x", self.start_x)
        self.start_y = data.get("start_y", self.start_y)
        self.start_z = data.get("start_z", self.start_z)
        self.end_x = data.get("end_x", self.end_x)
        self.end_y = data.get("end_y", self.end_y)
        self.end_z = data.get("end_z", self.end_z)
        self.radius = data.get("radius", self.radius)
        self.clockwise = data.get("clockwise", self.clockwise)
        self.num_segments = data.get("num_segments", self.num_segments)
        self.current_step = data.get("current_step", self.current_step)

        # Убедимся, что длина velocity и yaw соответствует num_segments
        self.velocity = self._adjust_list_length(data.get("velocity", []), self.num_segments, default_value=self.default_velocity)
        self.yaw = self._adjust_list_length(data.get("yaw", []), self.num_segments, default_value=self.default_yaw)

        # Переинициализация списка с обновленными данными
        self.init_trajectory_setpoint_list()

    def get_trajectory_setpoint(self) -> TrajectorySetpoint:
        """
        Возвращает текущий `TrajectorySetpoint` на основе текущего шага `current_step`.
        """
        if self.current_step < len(self.trajectory_setpoint_list):
            return self.trajectory_setpoint_list[self.current_step]
        else:
            self.complete = True
            self.current_step = 0
            return None

    def init_trajectory_setpoint_list(self):
        """
        Инициализация списка точек траектории с объектами TrajectorySetpoint.
        """
        self.trajectory_setpoint_list = []

        # Рассчитываем центр дуги
        mid_x = (self.start_x + self.end_x) / 2
        mid_y = (self.start_y + self.end_y) / 2
        direction = 1 if self.clockwise else -1

        # Определяем вектор от центра дуги к начальной точке
        dx = self.start_x - mid_x
        dy = self.start_y - mid_y
        distance = np.sqrt(dx ** 2 + dy ** 2)

        # Нормализуем вектор и масштабируем его до радиуса
        scale_factor = self.radius / distance
        center_x = mid_x + dx * scale_factor * direction
        center_y = mid_y + dy * scale_factor * direction
        center_z = self.start_z  # Предполагаем, что z остается постоянным

        # Расчет углов дуги
        start_angle = np.arctan2(self.start_y - center_y, self.start_x - center_x)
        end_angle = np.arctan2(self.end_y - center_y, self.end_x - center_x)

        # Корректируем направление углов для движения по часовой или против часовой стрелки
        if self.clockwise and end_angle > start_angle:
            end_angle -= 2 * np.pi
        elif not self.clockwise and end_angle < start_angle:
            end_angle += 2 * np.pi

        # Линейно распределяем углы между началом и концом
        angles = np.linspace(start_angle, end_angle, self.num_segments + 1)

        # Генерация точек траектории
        for i, angle in enumerate(angles):
            trajectory = TrajectorySetpoint()

            # Координаты точки на дуге
            trajectory.position[0] = center_x + self.radius * np.cos(angle)
            trajectory.position[1] = center_y + self.radius * np.sin(angle)
            trajectory.position[2] = self.start_z + (self.end_z - self.start_z) * (i / self.num_segments)

            # Устанавливаем скорость: используем значение из списка или default_velocity
            trajectory.velocity[0] = self.velocity[i] if i < len(self.velocity) else self.default_velocity
            trajectory.velocity[1] = float('nan')  # Оставляем NaN для Y и Z
            trajectory.velocity[2] = float('nan')

            # Устанавливаем yaw: используем значение из списка или default_yaw
            trajectory.yaw = self.yaw[i] if i < len(self.yaw) else self.default_yaw

            # Добавляем точку в список траекторий
            self.trajectory_setpoint_list.append(trajectory)

    def _adjust_list_length(self, input_list, target_length, default_value):
        """
        Приводит длину списка к target_length.
        Если список короче, добавляет default_value.
        Если длиннее, обрезает до target_length.
        """
        if not input_list:
            input_list = []
        if len(input_list) < target_length:
            input_list.extend([default_value] * (target_length - len(input_list)))
        elif len(input_list) > target_length:
            input_list = input_list[:target_length]
        return input_list


# G30: Спиральная траектория
class G30_SpiralMove(BaseGCommand):
    def __init__(self, 
                 counter, 
                 center_x, center_y, center_z, 
                 start_radius, end_radius, 
                 height, 
                 num_turns=1, 
                 num_segments=10, 
                 velocity=None, 
                 yaw=None, 
                 default_velocity=float('nan'), 
                 default_yaw=float('nan'), 
                 current_step=0):
        super().__init__("G30", counter)
        self.center_x = center_x
        self.center_y = center_y
        self.center_z = center_z
        self.start_radius = start_radius
        self.end_radius = end_radius
        self.height = height
        self.num_turns = num_turns
        self.num_segments = num_segments
        self.default_velocity = default_velocity
        self.default_yaw = default_yaw
        self.current_step = current_step

        # Убедимся, что длина velocity и yaw соответствует num_segments
        self.velocity = self._adjust_list_length(velocity, num_segments, default_value=self.default_velocity)
        self.yaw = self._adjust_list_length(yaw, num_segments, default_value=self.default_yaw)

        # Инициализируем список точек траектории
        self.init_trajectory_setpoint_list()

    def init_trajectory_setpoint_list(self):
        """
        Инициализация списка точек траектории с объектами TrajectorySetpoint.
        """
        self.trajectory_setpoint_list = []
        radii = np.linspace(self.start_radius, self.end_radius, self.num_segments + 1)
        heights = np.linspace(self.center_z, self.center_z + self.height, self.num_segments + 1)
        angles = np.linspace(0, 2 * np.pi * self.num_turns, self.num_segments + 1)

        for i, (radius, angle, height) in enumerate(zip(radii, angles, heights)):
            trajectory = TrajectorySetpoint()
            trajectory.position[0] = self.center_x + radius * np.cos(angle)
            trajectory.position[1] = self.center_y + radius * np.sin(angle)
            trajectory.position[2] = height

            trajectory.velocity[0] = self.velocity[i]
            trajectory.yaw = self.yaw[i]

            self.trajectory_setpoint_list.append(trajectory)

# G31: Обход вокруг точки
class G31_LoopAroundPoint(BaseGCommand):
    def __init__(self, 
                 counter, 
                 center_x, center_y, center_z, 
                 radius, 
                 num_turns=1, 
                 num_segments=10, 
                 velocity=None, 
                 yaw=None, 
                 default_velocity=float('nan'), 
                 default_yaw=float('nan'), 
                 current_step=0):
        super().__init__("G31", counter)
        self.center_x = center_x
        self.center_y = center_y
        self.center_z = center_z
        self.radius = radius
        self.num_turns = num_turns
        self.num_segments = num_segments
        self.default_velocity = default_velocity
        self.default_yaw = default_yaw
        self.current_step = current_step

        # Убедимся, что длина velocity и yaw соответствует num_segments
        self.velocity = self._adjust_list_length(velocity, num_segments, default_value=self.default_velocity)
        self.yaw = self._adjust_list_length(yaw, num_segments, default_value=self.default_yaw)

        # Инициализируем список точек траектории
        self.init_trajectory_setpoint_list()

    def init_trajectory_setpoint_list(self):
        """
        Инициализация списка точек траектории с объектами TrajectorySetpoint.
        """
        self.trajectory_setpoint_list = []
        angles = np.linspace(0, 2 * np.pi * self.num_turns, self.num_segments + 1)

        for i, angle in enumerate(angles):
            trajectory = TrajectorySetpoint()
            trajectory.position[0] = self.center_x + self.radius * np.cos(angle)
            trajectory.position[1] = self.center_y + self.radius * np.sin(angle)
            trajectory.position[2] = self.center_z

            trajectory.velocity[0] = self.velocity[i]
            trajectory.yaw = self.yaw[i]

            self.trajectory_setpoint_list.append(trajectory)

# G32: Плавное возвращение на базу
class G32_ReturnToBase(BaseGCommand):
    def __init__(self, 
                 counter, 
                 start_x, start_y, start_z, 
                 base_x, base_y, base_z, 
                 num_segments=10, 
                 velocity=None, 
                 yaw=None, 
                 default_velocity=float('nan'), 
                 default_yaw=float('nan'), 
                 current_step=0):
        super().__init__("G32", counter)
        self.start_x = start_x
        self.start_y = start_y
        self.start_z = start_z
        self.base_x = base_x
        self.base_y = base_y
        self.base_z = base_z
        self.num_segments = num_segments
        self.default_velocity = default_velocity
        self.default_yaw = default_yaw
        self.current_step = current_step

        # Убедимся, что длина velocity и yaw соответствует num_segments
        self.velocity = self._adjust_list_length(velocity, num_segments, default_value=self.default_velocity)
        self.yaw = self._adjust_list_length(yaw, num_segments, default_value=self.default_yaw)

        # Инициализируем список точек траектории
        self.init_trajectory_setpoint_list()

    def init_trajectory_setpoint_list(self):
        """
        Инициализация списка точек траектории с объектами TrajectorySetpoint.
        """
        self.trajectory_setpoint_list = []
        x_points = np.linspace(self.start_x, self.base_x, self.num_segments + 1)
        y_points = np.linspace(self.start_y, self.base_y, self.num_segments + 1)
        z_points = np.linspace(self.start_z, self.base_z, self.num_segments + 1)

        for i, (x, y, z) in enumerate(zip(x_points, y_points, z_points)):
            trajectory = TrajectorySetpoint()
            trajectory.position[0] = x
            trajectory.position[1] = y
            trajectory.position[2] = z

            trajectory.velocity[0] = self.velocity[i]
            trajectory.yaw = self.yaw[i]

            self.trajectory_setpoint_list.append(trajectory)