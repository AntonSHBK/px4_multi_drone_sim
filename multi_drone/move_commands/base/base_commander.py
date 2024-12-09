from abc import ABC, abstractmethod
from threading import Lock

from multi_drone.controllers.base.base_controller import BaseDroneController
from multi_drone.move_commands.base.base_g_code import BaseGCommand


class BaseCommander(ABC):
    """
    Базовый класс для командеров, управляющих командами дрона.
    """
    def __init__(self, controller: BaseDroneController):
        self.controller = controller
        self.command_queue = []
        self.command_history = []
        self.lock = Lock()
        self.active_command: BaseGCommand = None

    @abstractmethod
    def process_incoming_command(self, data: dict):
        """
        Метод для обработки входящей команды.
        :param data: Словарь данных команды.
        """
        pass

    def add_command(self, command):
        """
        Добавляет команду в очередь.
        :param command: Объект команды.
        """
        with self.lock:
            self.command_queue.append(command)
            self.controller.log_info(f"Команда {command.name} добавлена в очередь.")

    # def execute_commands(self):
    #     """
    #     Выполняет команды из очереди.
    #     """
    #     pass

    @abstractmethod
    def handle_completion(self, command):
        """
        Метод вызывается при завершении команды.
        """
        pass
