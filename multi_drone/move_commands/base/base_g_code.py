# base_command.py

from abc import ABC, abstractmethod

class BaseGCommand(ABC):
    def __init__(self, name, counter):
        self.name = name
        self.counter = counter
        self.complete = False        
        self.current_step = 0

    @abstractmethod
    def execute(self, controller):
        """
        Выполняет текущую команду, используя предоставленный контроллер.
        :param controller: Экземпляр контроллера для отправки команд на дрон.
        """
        pass

    @abstractmethod
    def is_complete(self, controller) -> bool:
        """
        Проверяет, завершена ли команда.
        :param controller: Экземпляр контроллера для проверки состояния дрона.
        :return: True, если команда завершена, иначе False.
        """
        pass

    def to_dict(self) -> dict:
        """
        Формирует словарь с параметрами команды.
        :return: Словарь с параметрами команды.
        """
        return {
            "name": self.name,
            "counter": self.counter,
            "complete": self.complete
        }

    @classmethod
    def from_dict(cls, data: dict):
        """
        Создает экземпляр команды из словаря.
        :param data: Словарь с параметрами команды.
        :return: Экземпляр команды.
        """
        return cls(name=data["name"], counter=data["counter"])

    def __repr__(self):
        """
        Представление команды в строковом формате для отладки.
        """
        return f"{self.__class__.__name__}(name={self.name}, counter={self.counter}, complete={self.complete})"
