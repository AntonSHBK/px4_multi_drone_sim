#!/usr/bin/env python

from typing import Literal

from launch_ros.actions import Node
from launch.actions import ExecuteProcess

class PX4Process:
    """
    Класс для создания процесса PX4 с возможностью выбора терминала или прямого запуска.
    """

    def __init__(
        self, 
        drone_id: int=1, 
        drone_type: str='x500', 
        spawn_position: list=[0, 0, 0, 0, 0, 0], 
        px4_autostart: int=4001,
        px4_dir: str = "/workspace/src/PX4-Autopilot/",
        terminal: Literal[
            'gnome-terminal', 'xterm', 'konsole', 'bash'
        ]='gnome-terminal',
    ):
        """
        :param drone_id: ID дрона.
        :param drone_type: Тип дрона (например, 'x500').
        :param spawn_position: Стартовая позиция дрона в формате "x,y,z,roll,pitch,yaw".
        :param px4_autostart: Автоматический запуск PX4 с заданным параметром.
        :param px4_dir: Путь к директории PX4.
        :param terminal: Терминал для запуска процесса. Возможные значения: 'gnome-terminal', 'xterm', 'konsole', None.
        """
        self.drone_id = drone_id
        self.drone_type = drone_type
        self.spawn_position = ','.join(map(str, spawn_position))
        self.px4_autostart = px4_autostart
        self.px4_dir = px4_dir
        self.terminal = terminal

    def create_process(self) -> ExecuteProcess:
        """
        Создает процесс PX4.

        :return: Объект ExecuteProcess для запуска PX4.
        """
        base_cmd = [
            'PX4_GZ_STANDALONE=1',
            f'PX4_GZ_MODEL_POSE="{self.spawn_position}"',
            f'PX4_SYS_AUTOSTART={self.px4_autostart}',
            f'PX4_GZ_MODEL={self.drone_type}',
            f"{self.px4_dir}/build/px4_sitl_default/bin/px4",
            f'-i {self.drone_id}'
        ]

        if self.terminal == 'gnome-terminal':
            cmd = ['gnome-terminal', '--', 'bash', '-c', ' '.join(base_cmd)]
        elif self.terminal == 'xterm':
            cmd = ['xterm', '-hold', '-e', ' '.join(base_cmd)]
        elif self.terminal == 'konsole':
            cmd = ['konsole', '--hold', '-e', ' '.join(base_cmd)]
        elif self.terminal == 'bash' or not self.terminal:
            cmd = ['bash', '-c', ' '.join(base_cmd)]
        else:
            raise ValueError(f"Неподдерживаемый терминал: {self.terminal}")

        return ExecuteProcess(cmd=cmd, output='screen')


class ControllerNode:
    """
    Универсальный класс для создания и управления нодой контроллера.
    """

    def __init__(
        self,
        package_name: str = 'multi_drone',
        controller_script: str = 'x500.py',
        drone_id: int = 1,
        drone_type: str = 'x500',
        spawn_position: list = [0, 0, 0, 0, 0, 0],
        output: str = 'log',
        additional_params: dict = {},
    ):
        """
        Инициализация контроллера.

        :param drone_id: Идентификатор дрона.
        :param drone_type: Тип дрона.
        :param spawn_position: Позиция [x, y, z, roll, pitch, yaw].
        :param output: Тип вывода ('log' или 'screen').
        :param package_name: Имя пакета.
        :param log_file: Имя файла для логов.
        :param additional_params: Дополнительные параметры для ноды.
        :param custom_executables: Кастомные исполняемые файлы для дронов.
        """
        self.package_name = package_name
        self.executables_script = controller_script
        self.drone_id = drone_id
        self.drone_type = drone_type
        self.default_position = [float(x) for x in spawn_position[:3]]
        self.default_orientation = [float(x) for x in spawn_position[3:]]
        self.output = output
        self.additional_params = additional_params

    def create_node(self) -> Node:
        """
        Создает и возвращает ноду контроллера.
        """
        parameters = [
            {'drone_id': self.drone_id},
            {'drone_type': self.drone_type},
            {'default_position': self.default_position},
            {'default_orientation': self.default_orientation},
            *[{key: value} for key, value in self.additional_params.items()]
        ]

        return Node(
            package=self.package_name,
            executable=self.executables_script,
            name=f'id_{self.drone_id}_{self.drone_type}_controller_node',
            output=self.output,
            parameters=parameters,
        )

def get_microxrce_agent_exec(
        udp: str = 'udp4',
        port: str = '8888'
    ) -> ExecuteProcess:
    return ExecuteProcess(
        cmd=[
                'MicroXRCEAgent', f'{udp}', '-p', f'{port}'
            ],
        output='log',
        name="microxrce_agent"
    )


def launch_robot(
        drone_id: int=1, 
        drone_type: str='x500', 
        spawn_position: str=[0, 0, 0, 0, 0, 0],  
        px4_autostart: int = 4001,
        px4_dir: str = "/workspace/src/PX4-Autopilot/",
        terminal: Literal[
            'gnome-terminal', 'xterm', 'konsole', 'bash'
        ]='gnome-terminal',
        package_name='multi_drone',
        controller_script='x500.py',
        additional_params={}
    ):
    """
    Формируем список процессов и нод для одного робота на основе конфигурации.
    """    
    px4_process = PX4Process(
        drone_id=drone_id,
        drone_type=drone_type,
        spawn_position=spawn_position,
        px4_autostart=px4_autostart,
        px4_dir=px4_dir,
        terminal=terminal
    )

    controller_node = ControllerNode(
        package_name=package_name,
        controller_script=controller_script,
        additional_params=additional_params,
        drone_id=drone_id,
        drone_type=drone_type,
        spawn_position=spawn_position,
        output='log'
    )

    nodes = [
        px4_process.create_process(),
        controller_node.create_node()
    ]

    return nodes

if __name__ == '__main__':
    launch_robot()
