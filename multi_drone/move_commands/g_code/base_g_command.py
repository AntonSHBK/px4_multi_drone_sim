# base_command.py

import json
from abc import ABC, abstractmethod

class BaseGCommand(ABC):
    def __init__(self, name, counter):
        self.name = name
        self.counter_command = counter
        self.complete = False
        
        self.current_step = 0
        self.trajectory_setpoint_list: list = None

    @abstractmethod
    def to_dict(self):
        """
        Формирует словарь из данных команды.
        """
        raise NotImplementedError("Метод to_dict должен быть реализован в дочернем классе")

    @abstractmethod
    def from_dict(self, dict_data):
        """
        Заполняет атрибуты команды данными из словаря.
        """
        raise NotImplementedError("Метод from_dict должен быть реализован в дочернем классе")
    
    @abstractmethod
    def get_trajectory_setpoint(self):
        """
        Возвращает объект TrajectorySetpoint
        """
        raise NotImplementedError("Метод get_trajectory_setpoint должен быть реализован в дочернем классе")  
    
    @abstractmethod
    def init_trajectory_setpoint_list(self):
        """
        Определяет объект self.trajectory_setpoint_list список TrajectorySetpoint
        """
        raise NotImplementedError("Метод get_trajectory_setpoints_list должен быть реализован в дочернем классе")  
    
    def step_complete(self):
        """Переход к следующему шагу или завершение команды, если шагов больше нет."""
        self.current_step += 1
        if self.trajectory_setpoint_list and self.current_step >= len(self.trajectory_setpoint_list):
            self.complete = True
            # self.current_step = 0

    def __repr__(self):
        return f"{self.__class__.__name__} (command_name={self.name})"
