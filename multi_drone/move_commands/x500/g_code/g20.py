import numpy as np

from multi_drone.utils.geometry import calculate_distance
from multi_drone.move_commands.x500.g_code.base_move_command import BaseMoveGCommand

from typing import TYPE_CHECKING, Literal, Optional
if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G20_MoveToPoint(BaseMoveGCommand):
    """
    Команда для перемещения дрона к одной заданной точке с возможностью указания угла ориентации (yaw) и скорости.
    """
    def __init__(self, 
                 counter: int = 0, 
                 x: Optional[float] = None, 
                 y: Optional[float] = None, 
                 z: Optional[float] = None, 
                 yaw: Optional[float] = None,
                 velocity: Optional[float] = None,
                 coordinate_system: Literal[
                    "local_NED", "local_ENU", 'global_ENU', 'global_NED'
                 ] = 'global_ENU',
                 current_step: int = 0):
        """
        :param counter: Счётчик команды.
        :param x: Координата X целевой точки.
        :param y: Координата Y целевой точки.
        :param z: Координата Z целевой точки.
        :param yaw: Угол ориентации в радианах.
        :param velocity: Скорость перемещения (м/с).
        :param coordinate_system: Система координат для перемещения.
        """
        super().__init__("G20", counter, coordinate_system, current_step)
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.velocity = velocity
        self.coordinate_system = coordinate_system
        self.update_targets_positions()

    def update_targets_positions(self):
        """
        Формирует список целевых позиций для команды.
        """
        self.targets_positions.clear()

        if self.x is not None and self.y is not None and self.z is not None:
            self.targets_positions.append(
                self.Position(
                    x=self.x,
                    y=self.y,
                    z=self.z,
                    velocity=self.velocity if self.velocity else 1.0,
                    yaw=self.yaw
                )
            )

    def to_dict(self) -> dict:
        """
        Сериализует команду в словарь.
        """
        base_dict = super().to_dict()
        base_dict.update({
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw": self.yaw,
            "velocity": self.velocity,
            "coordinate_system": self.coordinate_system,
        })
        return base_dict

    @classmethod
    def from_dict(cls, data: dict):
        """
        Десериализует команду из словаря.
        """
        return cls(
            counter=data.get("counter", 0),
            x=data.get("x", None),
            y=data.get("y", None),
            z=data.get("z", None),
            yaw=data.get("yaw", None),
            velocity=data.get("velocity", None),
            coordinate_system=data.get("coordinate_system", 'global_ENU'),
            current_step=data.get("current_step", 0)
        )

    def __repr__(self):
        """
        Строковое представление команды.
        """
        return (f"G20_MoveToPoint(counter={self.counter}, x={self.x}, y={self.y}, "
                f"z={self.z}, yaw={self.yaw}, velocity={self.velocity}, complete={self.complete})")
