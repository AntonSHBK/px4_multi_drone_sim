import numpy as np

from multi_drone.move_commands.x500.g_code.base_move_command import BaseMoveGCommand

from typing import TYPE_CHECKING, List, Literal, Optional
if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G24_SpiralTrajectory(BaseMoveGCommand):
    """
    Команда для движения дрона по спирали (по конусу, цилиндру и т.д.).
    """
    def __init__(self, 
                 counter: int = 0,
                 center_point: Optional[List[float]] = None,
                 radius_start: float = 1.0,
                 radius_end: float = 2.0,
                 height_change: float = 5.0,  # Изменение высоты за весь спиральный путь
                 turns: int = 3,  # Количество витков
                 direction: Literal["CW", "CCW"] = "CW",  # Clockwise, Counterclockwise
                 points_per_turn: int = 100,  # Количество точек на один виток
                 velocity: Optional[float] = None,
                 yaw_mode: Literal["fixed", "facing_center"] = "facing_center",
                 coordinate_system: Literal[
                    "local_NED", "local_ENU", "global_ENU", "global_NED"
                 ] = "global_ENU",
                 current_step: int = 0):
        """
        :param counter: Счётчик команды.
        :param center_point: Центр спирали [x, y, z].
        :param radius_start: Начальный радиус спирали.
        :param radius_end: Конечный радиус спирали.
        :param height_change: Изменение высоты за весь путь (может быть 0 для цилиндра).
        :param turns: Количество витков спирали.
        :param direction: Направление вращения ("CW" - по часовой, "CCW" - против часовой).
        :param points_per_turn: Количество точек на один виток.
        :param velocity: Линейная скорость (м/с).
        :param yaw_mode: Режим ориентации ("fixed" - фиксированный, "facing_center" - смотрит в центр спирали).
        :param coordinate_system: Система координат.
        """
        super().__init__("G24", counter, coordinate_system, current_step)
        self.center_point = np.array(center_point) if center_point else None
        self.radius_start = radius_start
        self.radius_end = radius_end
        self.height_change = height_change
        self.turns = turns
        self.direction = direction
        self.points_per_turn = points_per_turn
        self.velocity = velocity
        self.yaw_mode = yaw_mode
        self.update_targets_positions()

    def update_targets_positions(self):
        """
        Формирует список точек на спирали вокруг центра.
        """
        self.targets_positions.clear()

        if self.center_point is None:
            raise ValueError("Для выполнения спирали необходимо указать центр (center_point).")

        # Генерация углов для всех витков
        total_points = self.points_per_turn * self.turns
        angles = np.linspace(0, 2 * np.pi * self.turns, total_points)

        # Корректируем направление
        if self.direction == "CW":
            angles = -angles

        # Генерация радиуса и высоты для каждой точки
        radii = np.linspace(self.radius_start, self.radius_end, total_points)
        heights = np.linspace(self.center_point[2], self.center_point[2] + self.height_change, total_points)

        for i, angle in enumerate(angles):
            x = self.center_point[0] + radii[i] * np.cos(angle)
            y = self.center_point[1] + radii[i] * np.sin(angle)
            z = heights[i]

            # Рассчитываем yaw
            if self.yaw_mode == "facing_center":
                yaw = np.arctan2(self.center_point[1] - y, self.center_point[0] - x)
            else:
                yaw = None

            self.targets_positions.append(
                self.Position(x=x, y=y, z=z, velocity=self.velocity, yaw=yaw)
            )

    def to_dict(self) -> dict:
        """
        Сериализует команду в словарь.
        """
        base_dict = super().to_dict()
        base_dict.update({
            "center_point": self.center_point.tolist() if self.center_point is not None else None,
            "radius_start": self.radius_start,
            "radius_end": self.radius_end,
            "height_change": self.height_change,
            "turns": self.turns,
            "direction": self.direction,
            "points_per_turn": self.points_per_turn,
            "velocity": self.velocity,
            "yaw_mode": self.yaw_mode,
            "coordinate_system": self.coordinate_system
        })
        return base_dict

    @classmethod
    def from_dict(cls, data: dict):
        """
        Десериализует команду из словаря.
        """
        return cls(
            counter=data.get("counter", 0),
            center_point=data.get("center_point"),
            radius_start=data.get("radius_start", 1.0),
            radius_end=data.get("radius_end", 2.0),
            height_change=data.get("height_change", 5.0),
            turns=data.get("turns", 3),
            direction=data.get("direction", "CW"),
            points_per_turn=data.get("points_per_turn", 100),
            velocity=data.get("velocity"),
            yaw_mode=data.get("yaw_mode", "facing_center"),
            coordinate_system=data.get("coordinate_system", "global_ENU"),
            current_step=data.get("current_step", 0)
        )

    def __repr__(self):
        """
        Строковое представление команды.
        """
        return (f"G24_SpiralTrajectory(counter={self.counter}, center_point={self.center_point}, "
                f"radius_start={self.radius_start}, radius_end={self.radius_end}, "
                f"height_change={self.height_change}, turns={self.turns}, "
                f"direction={self.direction}, points_per_turn={self.points_per_turn}, "
                f"velocity={self.velocity}, yaw_mode={self.yaw_mode}, complete={self.complete})")
