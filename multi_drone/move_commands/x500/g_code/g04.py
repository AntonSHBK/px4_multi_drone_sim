from multi_drone.controllers.x500.states import LoiterState, OffboardState, LandingState
from multi_drone.move_commands.base.base_g_code import BaseGCommand

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G4_Land(BaseGCommand):
    """
    Команда для автоматической посадки дрона.
    """
    def __init__(self, counter: int = 0):
        """
        :param counter: Счётчик команды.
        """
        super().__init__("G4", counter)

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, можно ли выполнить команду.
        Условие: дрон должен находиться в состоянии LoiterState или OffboardState.
        """
        if isinstance(controller.current_state, (LoiterState, OffboardState)):
            return True

        controller.log_error(
            "G4_Land: Команда может быть выполнена только из LoiterState или OffboardState."
        )
        return False

    def execute(self, controller: 'X500Controller'):
        """
        Инициирует процесс посадки дрона.
        """
        controller.params.offboard_mode = False
        controller.params.landing = True
        # controller.log_info("G4_Land: Запрос на посадку отправлен.")

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, завершена ли команда посадки.
        Условие: текущее состояние должно быть LandingState.
        """
        if isinstance(controller.current_state, LandingState):
            # controller.log_info("G4_Land: Дрон находится в состоянии LandingState. Посадка началась.")
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
        return f"G4_Land(counter={self.counter}, complete={self.complete})"
