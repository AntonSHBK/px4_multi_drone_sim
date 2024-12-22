from multi_drone.controllers.x500.states import ArmingState, LoiterState
from multi_drone.move_commands.base.base_g_code import BaseGCommand

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G3_Takeoff(BaseGCommand):
    """
    Команда для автоматического взлёта дрона.
    """
    def __init__(self, counter: int = 0, altitude: float = 2.5):
        """
        :param counter: Счётчик команды.
        :param altitude: Целевая высота взлёта (в метрах).
        """
        super().__init__("G3", counter)
        self.altitude = altitude

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, можно ли выполнить команду.
        Условие: дрон должен быть в состоянии ArmingState.
        """
        if not isinstance(controller.current_state, ArmingState):
            controller.log_error("G3_Takeoff: Команда может быть выполнена только из состояния ArmingState.")
            return False
        return True

    def execute(self, controller: 'X500Controller'):
        """
        Устанавливает цель для взлёта на заданную высоту.
        """
        controller.params.landing = False
        controller.params.takeoff = True
        # controller.log_info(f"G3_Takeoff: Инициирован взлёт на высоту {self.altitude} м.")

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, успешно ли выполнен взлёт.
        Условие: текущее состояние должно быть LoiterState.
        """
        if isinstance(controller.current_state, LoiterState):
            # controller.log_info("G3_Takeoff: Дрон успешно перешёл в состояние LoiterState.")
            self.complete_command()
        return self._check_finish()

    def to_dict(self) -> dict:
        """
        Сериализует команду в словарь.
        """
        base_dict = super().to_dict()
        base_dict.update({"altitude": self.altitude})
        return base_dict

    @classmethod
    def from_dict(cls, data: dict):
        """
        Десериализует команду из словаря.
        """
        return cls(counter=data.get("counter", 0), altitude=data.get("altitude", 2.5))

    def __repr__(self):
        """
        Строковое представление команды.
        """
        return f"G3_Takeoff(counter={self.counter}, altitude={self.altitude}, complete={self.complete})"
