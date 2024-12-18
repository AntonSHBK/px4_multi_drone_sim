import json
import inspect

from std_msgs.msg import String

from multi_drone.move_commands.base.base_commander import DroneCommander
from multi_drone.move_commands.base.base_g_code import BaseGCommand
import multi_drone.move_commands.x500.g_code as g_code_module

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from multi_drone.controllers.x500.x500 import X500Controller

class X500Commander(DroneCommander):
    """
    Коммандер для дрона X500.
    """

    def __init__(self, controller:  "X500Controller", timer_execution=0.1):
        super().__init__(controller)        

        self.command_classes = {
            cls("temp").name: cls
            for _, cls in inspect.getmembers(g_code_module, inspect.isclass)
            if issubclass(cls, BaseGCommand) and cls != BaseGCommand
        }

        self.subscriber_command_json = controller.create_subscription(
            String,
            f"{controller.prefix_px}/command_json",
            self.command_json_callback,
            controller.qos_profile_reliable
        )
        
        self.timer_execution_commands = controller.create_timer(timer_execution, self._timer_execute_commands_callback)

    def _timer_execute_commands_callback(self):
        """
        ROS-таймер для выполнения команд.
        """
        
        if self.active_command:
            try:
                if self.active_command.is_complete(self.controller):
                    self.handle_completion(self.active_command)
                    self.active_command = None
            except Exception as e:
                self.controller.log_error(f"Ошибка при проверке завершения команды: {e}")
                self.active_command = None

        elif self.command_queue:
            self.active_command = self.command_queue.pop(0)
            self.controller.log_info(f"Выполнение команды: {self.active_command.name}")
            self.active_command.safe_execute(self.controller)

    def command_json_callback(self, msg: String):
        """
        Обработчик входящих ROS-команд.
        """
        try:
            data = json.loads(msg.data)
            self.process_incoming_command(data)
        except json.JSONDecodeError as e:
            self.controller.log_error(f"Ошибка декодирования JSON: {e}")

    def process_incoming_command(self, data: dict):
        """
        Обработка входящей команды.
        """
        command_name = data.get("name")
        if not command_name:
            self.controller.log_error("Отсутствует имя команды в данных.")
            return
        
        command_class: BaseGCommand = self.command_classes.get(command_name)

        if not command_class:
            self.controller.log_error(f"Неизвестная команда: {command_name}")
            return

        command = command_class.from_dict(data)
        self.command_history.append(data)
        self.add_command(command)

    def handle_completion(self, command: BaseGCommand):
        """
        Вызывается при завершении команды.
        """
        self.controller.log_info(f"Команда {command.name} завершена.")
