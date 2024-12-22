import numpy as np

from multi_drone.controllers.x500.states import OffboardState
from multi_drone.move_commands.base.base_g_code import BaseGCommand
from multi_drone.utils.geometry import calculate_distance

from typing import TYPE_CHECKING, List, Optional
if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller
  

class BaseMoveGCommand(BaseGCommand):
    """
    Базовый класс для команд перемещения дрона.
    """
    class Position:
        """
        Класс для описания позиции с координатами, скоростью и ориентацией yaw.
        """
        def __init__(self, x: float, y: float, z: float, velocity: float, yaw: Optional[float]):
            self.x = x
            self.y = y
            self.z = z
            self.velocity = velocity
            self.yaw = yaw
            
        def position_to_array(self):
            return np.array([self.x, self.y, self.z])

        def __repr__(self):
            return f"Position(x={self.x}, y={self.y}, z={self.z}, velocity={self.velocity}, yaw={self.yaw})"

    def __init__(
        self, 
        name: str, 
        counter: int = 0, 
        coordinate_system: str = "global_ENU",
        current_step: int = 0
    ):
        super().__init__(name, counter, current_step)
        self.targets_positions: List[BaseMoveGCommand.Position] = []
        self.coordinate_system = coordinate_system

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, находится ли дрон в режиме offboard.
        """
        if not isinstance(controller.current_state, OffboardState):
            controller.log_error(f"{self.name}: Команду можно выполнить только из состояния OffboardState.")
            return False
        return True

    def update_targets_positions(self):
        """
        Формирует список целевых позиций для выполнения команды.
        Должен быть реализован в дочерних классах.
        """
        raise NotImplementedError("Метод update_targets_positions() должен быть реализован в дочернем классе.")

    def execute(self, controller: 'X500Controller'):
        """
        Выполняет команду для текущего этапа.
        """
        if self.current_step < len(self.targets_positions):
            target: BaseMoveGCommand.Position = self.targets_positions[self.current_step]
            controller.offboard_commander.update(
                position=target.position_to_array(),
                # velocity=[target.velocity, 0, 0],
                yaw=target.yaw,
                system=self.coordinate_system,
                # mode="position"
            )

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Проверяет завершение текущего этапа и переключает на следующий.
        """
        if not self.targets_positions:
            self.mark_as_interrupted()
            return self._check_finish()
        
        if self.current_step < len(self.targets_positions):            
            current_position = controller.current_position.get_position(
                system=self.coordinate_system)
            target_position = self.targets_positions[self.current_step]            
            target_position_point = target_position.position_to_array()
            
            yaw_tolerance = 0.5  # Радианы
            position_tolerance = 0.5  # Метры
            
            target_position = np.array(
                [v if v is not None else np.nan for v in target_position_point]
            )
            mask = ~np.isnan(target_position_point)
            
            masked_current_position = current_position[mask]
            masked_target_position_point = target_position_point[mask]

            position_error = calculate_distance(
                masked_current_position, masked_target_position_point
            )
            
            # if self.yaw is not None:
            #     current_yaw = controller.current_position.get_orientation(system=self.coordinate_system)
            #     yaw_error = abs((current_yaw - self.yaw + np.pi) % (2 * np.pi) - np.pi)
            #     if yaw_error > yaw_tolerance:
            #         return False

            
            if position_error <= position_tolerance:
                self.current_step += 1
                if self.current_step < len(self.targets_positions):
                    self.execute(controller)
                else:
                    self.complete_command()
        return self._check_finish()
