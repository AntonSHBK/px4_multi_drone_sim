from abc import ABC, abstractmethod
from typing import Type
from threading import Lock

from multi_drone.controllers.base.base_controller import BaseDroneController
from multi_drone.move_commands.base.base_g_code import BaseGCommand


class DroneCommander(ABC):
    """
    Базовый класс для командеров, управляющих командами дрона.
    """
    def __init__(self, controller: BaseDroneController):
        self.controller = controller
        self.command_classes = {}
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

    def add_command(self, command: BaseGCommand):
        """
        Добавляет команду в очередь.
        :param command: Объект команды.
        """
        with self.lock:
            self.command_queue.append(command)

    @abstractmethod
    def execute(self, controller: 'BaseDroneController'):
        """
        Выполняет команды из очереди.
        """
        pass

    @abstractmethod
    def handle_completion(self, command):
        """
        Метод вызывается при завершении команды.
        """
        pass

    def clear_command_queue(self):
        """
        Метод очистки команд из очереди.
        """
        with self.lock:
            self.command_queue.clear()        
        
    def add_command_class(self, name: str, class_type: Type[object]) -> None:
        """
        Добавляет ключ и класс в словарь.

        :param class_dict: Словарь, где ключи - это названия классов, а значения - классы.
        :param name: Название класса для ключа.
        :param class_type: Класс Python, который будет добавлен в словарь.
        """
        if name in self.command_classes:
            raise ValueError(f"Класс с именем '{name}' уже существует в словаре.")
        self.command_classes[name] = class_type