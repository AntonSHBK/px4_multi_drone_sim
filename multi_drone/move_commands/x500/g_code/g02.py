from multi_drone.controllers.x500.states import ArmingState, LandingState, IdleState
from multi_drone.move_commands.base.base_g_code import BaseGCommand

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G2_Disarm(BaseGCommand):
    """
    Команда для разармирования дрона (отключение моторов).
    """
    def __init__(self, counter: int = 0):
        """
        :param counter: Счётчик команды.
        """
        super().__init__("G2", counter)

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, можно ли выполнить команду.
        Условие: текущее состояние должно быть ArmingState или LandingState.
        """
        if isinstance(controller.current_state, (ArmingState, LandingState)):
            return True

        controller.log_error(
            "G2_Disarm: Команда может быть выполнена только из состояний ArmingState или LandingState."
        )
        return False

    def execute(self, controller: 'X500Controller'):
        """
        Выполняет разармирование дрона.
        """
        controller.params.arming = False
        # controller.log_info("G2_Disarm: Запрос на разармирование отправлен.")

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, успешно ли выполнено разармирование дрона.
        Условие: текущее состояние должно быть DisarmState.
        """
        if isinstance(controller.current_state, IdleState):
            # controller.log_info("G2_Disarm: Дрон успешно разармирован (DisarmState).")
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
        return f"G2_Disarm(counter={self.counter}, complete={self.complete})"
