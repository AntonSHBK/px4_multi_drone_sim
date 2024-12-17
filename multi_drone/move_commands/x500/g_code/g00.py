from multi_drone.move_commands.base.base_g_code import BaseGCommand

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller


class G0_Stop(BaseGCommand):
    """
    Команда для немедленного останова всех движений дрона.
    """
    def __init__(self, counter: int = 0):
        super().__init__("G0", counter)

    def can_execute(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, можно ли выполнить команду.
        """
        return True

    def execute(self, controller: 'X500Controller'):
        """
        Выполняет останов дрона, обнуляя все целевые скорости.
        """
        controller.offboard_commander.update()
        controller.log_info("G0_Stop: Все движения дрона прекращены.")

    def is_complete(self, controller: 'X500Controller') -> bool:
        """
        Проверяет, достиг ли дрон состояния покоя.
        """
        current_velocity = controller.current_position.get_velocity(system='local_ENU')
        velocity_tolerance = 0.05  # Допустимая погрешность по скорости (м/с)

        # Проверка, что скорость по всем осям близка к нулю
        if all(abs(v) <= velocity_tolerance for v in current_velocity):
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
        return f"G0_Stop(counter={self.counter}, complete={self.complete})"
