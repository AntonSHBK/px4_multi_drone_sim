# g_code.py

import numpy as np

from multi_drone.move_commands.base.base_g_code import BaseGCommand
from multi_drone.controllers.x500.states import OffboardState

from multi_drone.utils.geometry import calculate_distance

from typing import TYPE_CHECKING, Literal
if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G20_MoveToPoint(BaseGCommand):
    """
    Команда для перемещения дрона к одной заданной точке с возможностью указания угла ориентации (yaw) и скорости.
    """
    def __init__(self, 
                 counter: int = 0, 
                 x: float = None, 
                 y: float = None, 
                 z: float = None, 
                 yaw: float = None,
                 velocity: float = None,
                 coordinate_system: Literal[
                    "local_NED", "local_ENU", 'global_ENU', 'global_NED'
                ] = 'global_ENU'):
        super().__init__("G20", counter)
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.velocity = velocity
        self.coordinate_system = coordinate_system

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, находится ли дрон в режиме offboard.
        """
        if not isinstance(controller.current_state, OffboardState):
            controller.log_error("G20_MoveToPoint: Команду можно выполнить только из состояния OffboardState.")
            return False

        return True

    def execute(self, controller: 'X500Controller'):
        """
        Отправляет команду перемещения к указанной точке через контроллер.
        """
        target_position = [self.x, self.y, self.z]
        
        controller.offboard_commander.update(
            position=target_position,
            yaw=self.yaw,
            system=self.coordinate_system,
            mode='position'
        )

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, достиг ли дрон целевой точки.
        """
        current_position = controller.current_position.get_position(system=self.coordinate_system)
        target_position = np.array([self.x, self.y, self.z], dtype=np.float32)
        
        yaw_tolerance = 0.1  # Радианы
        position_tolerance = 0.1  # Метры

        target_position = np.array([v if v is not None else np.nan for v in target_position])
        mask = ~np.isnan(target_position)
        masked_current_position = current_position[mask]
        masked_target_position = target_position[mask]

        position_error = calculate_distance(masked_current_position, masked_target_position)
        if position_error > position_tolerance:
            return False

        if self.yaw is not None:
            current_yaw = controller.current_position.get_orientation(system=self.coordinate_system)
            yaw_error = abs((current_yaw - self.yaw + np.pi) % (2 * np.pi) - np.pi)
            if yaw_error > yaw_tolerance:
                return False

        self.complete = True
        return True

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
            x=data.get("x", None),
            y=data.get("y", None),
            z=data.get("z", None),
            yaw=data.get("yaw", None),
            velocity=data.get("velocity", None),
            coordinate_system=data.get("coordinate_system", 'global_ENU')
        )

    def __repr__(self):
        """
        Строковое представление команды.
        """
        return (f"G20_MoveToPoint(counter={self.counter}, x={self.x}, y={self.y}, "
                f"z={self.z}, yaw={self.yaw}, velocity={self.velocity}, complete={self.complete})")