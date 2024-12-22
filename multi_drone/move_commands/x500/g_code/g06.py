from multi_drone.controllers.x500.states import LoiterState, OffboardState
from multi_drone.move_commands.base.base_g_code import BaseGCommand

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G6_Offboard(BaseGCommand):
    """
    Команда для активации режима Offboard.
    """
    def __init__(self, counter: int = 0):
        """
        :param counter: Счётчик команды.
        """
        super().__init__("G6", counter)

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, можно ли выполнить команду.
        Условие: текущее состояние должно быть LoiterState.
        """
        if isinstance(controller.current_state, LoiterState):
            return True

        controller.log_error("G6_Offboard: Команда может быть выполнена только из состояния LoiterState.")
        return False

    def execute(self, controller: 'X500Controller'):
        """
        Устанавливает режим Offboard через систему статусов.
        """
        controller.params.offboard_mode = True
        controller.params.landing = False
        # controller.log_info("G6_Offboard: Запрос на активацию режима Offboard отправлен.")

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, успешно ли активирован режим Offboard.
        Условие: текущее состояние должно быть OffboardState.
        """
        if isinstance(controller.current_state, OffboardState):
            # controller.log_info("G6_Offboard: Дрон успешно переключен в состояние OffboardState.")
            self.complete_command()
        return self._check_finish()

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
        return f"G6_Offboard(counter={self.counter}, complete={self.complete})"
