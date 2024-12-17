from multi_drone.controllers.x500.states import OffboardState, LoiterState
from multi_drone.move_commands.base.base_g_code import BaseGCommand

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G5_Loiter(BaseGCommand):
    """
    Команда для перевода дрона в режим зависания (Loiter).
    """
    def __init__(self, counter: int = 0):
        """
        :param counter: Счётчик команды.
        """
        super().__init__("G5", counter)

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, можно ли выполнить команду.
        Условие: текущее состояние должно быть OffboardState.
        """
        if not isinstance(controller.current_state, OffboardState):
            controller.log_error("G5_Loiter: Команду можно выполнить только из состояния OffboardState.")
            return False

        return True

    def execute(self, controller: 'X500Controller'):
        """
        Переводит дрон в режим Loiter, установив параметры.
        """
        controller.params.offboard_mode = False
        controller.params.landing = False
        # controller.log_info("G5_Loiter: Перевод в режим Loiter инициирован.")

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, успешно ли дрон перешёл в режим LoiterState.
        """
        if isinstance(controller.current_state, LoiterState):
            # controller.log_info("G5_Loiter: Дрон успешно перешёл в режим LoiterState.")
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
        return f"G5_Loiter(counter={self.counter}, complete={self.complete})"
