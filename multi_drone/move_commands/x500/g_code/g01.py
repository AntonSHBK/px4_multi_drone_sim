from multi_drone.controllers.x500.states import IdleState, ArmingState
from multi_drone.move_commands.base.base_g_code import BaseGCommand

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G1_Arm(BaseGCommand):
    """
    Команда для армирования дрона (включение моторов).
    """
    def __init__(self, counter: int = 0):
        super().__init__("G1", counter)

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, можно ли выполнить команду.
        Условие: дрон должен находиться в состоянии Idle.
        """
        if not isinstance(controller.current_state, IdleState):
            controller.log_error("G1_Arm: Невозможно выполнить команду. Дрон не в состоянии Idle.")
            return False

        return True

    def execute(self, controller: 'X500Controller'):
        """
        Выполняет армирование дрона, отправляя сообщение для смены состояния.
        """
        controller.params.arming = True
        # controller.log_info("G1_Arm: Запрос на армирование отправлен.")

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, успешно ли выполнено армирование дрона.
        Условие: текущее состояние должно быть ArmingState.
        """
        if isinstance(controller.current_state, ArmingState):
            # controller.log_info("G1_Arm: Дрон успешно армирован и находится в состоянии ArmingState.")
            self.complete = True
        return self.complete

    def to_dict(self) -> dict:
        """
        Сериализует команду в словарь.
        """
        return super().to_dict()

    @classmethod
    def from_dict(cls, data: dict):
        """
        Десериализует команду из словаря.
        """
        return cls(counter=data.get("counter", 0))

    def __repr__(self):
        """
        Строковое представление команды.
        """
        return f"G1_Arm(counter={self.counter}, complete={self.complete})"
