# base_command.py

from abc import ABC, abstractmethod

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from multi_drone.controllers.base.base_controller import BaseDroneController

class BaseGCommand(ABC):
    def __init__(self, name, counter=0, current_step=0):
        self.name = name
        self.counter = counter
        self.complete = False
        self.interrupt = False
        self.current_step = current_step

    @abstractmethod
    def execute(self, controller):
        """
        Выполняет текущую команду, используя предоставленный контроллер.
        :param controller: Экземпляр контроллера для отправки команд на дрон.
        """
        pass

    @abstractmethod
    def is_complete(self, controller: 'BaseDroneController') -> bool:
        """
        Проверяет, завершена ли команда.
        :param controller: Экземпляр контроллера для проверки состояния дрона.
        :return: True, если команда завершена, иначе False.
        """
        pass

    @abstractmethod
    def can_execute(self, controller: 'BaseDroneController') -> bool:
        """
        Проверяет, можно ли выполнить команду.
        :param controller: Экземпляр контроллера для проверки условий выполнения.
        :return: True, если команда может быть выполнена, иначе False.
        """
        pass

    def safe_execute(self, controller: 'BaseDroneController'):
        """
        Проверяет возможность выполнения команды и завершённость перед вызовом execute.
        :param controller: Экземпляр контроллера.
        """
        if self._check_finish():
            # controller.log_info(f"Команда {self.name} уже завершена.")
            return
        if not self.can_execute(controller):
            # controller.log_error(f"Команда {self.name} не может быть выполнена в текущем состоянии.")
            self.mark_as_interrupted()
            return
        self.execute(controller)
        
    def _check_finish(self):
        if self.complete or self.interrupt:
            return True
        else:
            return False
    
    def mark_as_interrupted(self):
        """
        Устанавливает флаг, что команда должна быть прервана.
        """
        self.interrupt = True
        
    def complete_command(self):
        """
        Устанавливает флаг, что команда завершена.
        """
        self.complete = True

    def to_dict(self) -> dict:
        """
        Формирует словарь с параметрами команды.
        :return: Словарь с параметрами команды.
        """
        return {
            "name": self.name,
            "counter": self.counter,
            "current_step": self.current_step
        }

    @classmethod
    def from_dict(cls, data: dict):
        """
        Создает экземпляр команды из словаря.
        :param data: Словарь с параметрами команды.
        :return: Экземпляр команды.
        """
        return cls(
            name=data["name"], 
            counter=data["counter"], 
            current_step=data["current_step"]
        )

    def __repr__(self):
        """
        Представление команды в строковом формате для отладки.
        """
        return f"{self.__class__.__name__}(name={self.name}, counter={self.counter}, complete={self.complete})"
