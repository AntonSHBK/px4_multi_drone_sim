#!/usr/bin/env python

from typing import List, Literal, Optional

from launch_ros.actions import Node
from launch.actions import ExecuteProcess

class PX4Process:
    """
    Класс для создания и управления процессами PX4 с гибкими настройками.

    Параметры:
    ----------
    - drone_id : int
        Уникальный идентификатор для экземпляра дрона. По умолчанию 1.
    - drone_type : str
        Тип дрона (например, 'x500'). По умолчанию 'x500'.
    - gz_world : str
        Название файла мира Gazebo. По умолчанию 'default'.
    - spawn_position : List[float]
        Начальная позиция и ориентация дрона в формате [x, y, z, roll, pitch, yaw].
        По умолчанию [0, 0, 0, 0, 0, 0].
    - px4_autostart : int
        Идентификатор PX4 SYS_AUTOSTART для конфигурации. По умолчанию 4001.
    - px4_dir : str
        Путь к директории установки PX4. По умолчанию "/workspace/src/PX4-Autopilot/".
    - px4_simulator : str
        Тип симулятора (например, 'GZ' для Gazebo). По умолчанию 'GZ'.
    - px4_model_name : Optional[str]
        Имя существующей модели Gazebo, к которой подключается PX4. Взаимоисключается с px4_sim_model. Если оно указано, сценарий запуска пытается привязать новый экземпляр PX4 к ресурсу Gazebo с таким же именем.
    - px4_sim_model : Optional[str]
        Имя новой модели Gazebo, которая будет создана. Взаимоисключается с px4_model_name.  Если указано, сценарий запуска ищет модель в пути к ресурсам Gazebo, которая соответствует заданной переменной, создаёт её и привязывает к ней новый экземпляр PX4.
    - px4_render_engine : Optional[str]
        Механизм рендеринга для Gazebo. По умолчанию 'ogre'.
    - terminal : Literal['gnome-terminal', 'xterm', 'konsole', 'bash']
        Эмулятор терминала для запуска процесса. По умолчанию 'gnome-terminal'.

    Исключения:
    -----------
    - ValueError:
        Если px4_model_name и px4_sim_model указаны одновременно, так как они взаимно исключают друг друга.
    - ValueError:
        Если указанный эмулятор терминала не поддерживается.
    """

    def __init__(
        self,
        drone_id: int = 1,
        gz_world: str = "default",        
        px4_autostart: int = 4001,
        px4_dir: str = "/workspace/src/PX4-Autopilot/",
        px4_simulator: str = "GZ",
        px4_model_name: Optional[str] = None,
        px4_sim_model: Optional[str] = "x500",
        px4_gz_model_pose: List[float] = [0, 0, 0, 0, 0, 0],
        px4_render_engine: Optional[str] = "ogre",
        standalone: bool = True,
        terminal: Literal["gnome-terminal", "xterm", "konsole", "bash"] = "gnome-terminal",
    ):
        if px4_model_name and px4_sim_model:
            raise ValueError("px4_model_name и px4_sim_model взаимно исключают друг друга.")
        
        self.drone_id = drone_id
        self.gz_world = gz_world
        self.px4_gz_model_pose = ','.join(map(str, px4_gz_model_pose))
        self.px4_autostart = px4_autostart
        self.px4_dir = px4_dir
        self.px4_simulator = px4_simulator
        self.px4_model_name = px4_model_name
        self.px4_sim_model = px4_sim_model
        self.px4_render_engine = px4_render_engine
        self.standalone = standalone
        self.terminal = terminal

    def create_process(self) -> ExecuteProcess:
        """
        Создаёт процесс PX4 с заданными переменными окружения.

        Возвращает:
        -----------
        ExecuteProcess:
            Действие для запуска процесса PX4.
        """
        env_vars = [
            f"PX4_SYS_AUTOSTART={self.px4_autostart}",
            f"PX4_SIMULATOR={self.px4_simulator}",
            f"PX4_GZ_WORLD={self.gz_world}",
        ]
        
        if self.standalone:
            env_vars.append(f"PX4_GZ_STANDALONE=1")

        if self.px4_model_name:
            env_vars.append(f"PX4_GZ_MODEL_NAME={self.px4_model_name}")
        elif self.px4_sim_model:
            env_vars.extend([
                f"PX4_SIM_MODEL={self.px4_sim_model}",
                f"PX4_GZ_MODEL_POSE={self.px4_gz_model_pose}",
            ])

        if self.px4_render_engine:
            env_vars.append(f"PX4_GZ_SIM_RENDER_ENGINE={self.px4_render_engine}")

        cmd = [
            *env_vars,
            f"{self.px4_dir}/build/px4_sitl_default/bin/px4",
            f"-i {self.drone_id}",
        ]

        if self.terminal == "gnome-terminal":
            launch_cmd = ["gnome-terminal", "--", "bash", "-c", ' '.join(cmd)]
        elif self.terminal == "xterm":
            launch_cmd = ["xterm", "-hold", "-e", ' '.join(cmd)]
        elif self.terminal == "konsole":
            launch_cmd = ["konsole", "--hold", "-e", ' '.join(cmd)]
        elif self.terminal == "bash":
            launch_cmd = ["bash", "-c", ' '.join(cmd)]
        else:
            raise ValueError(f"Неподдерживаемый терминал: {self.terminal}")

        return ExecuteProcess(cmd=launch_cmd, output="screen")


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
        gz_world: str = 'default',
        spawn_position: str=[0, 0, 0, 0, 0, 0],  
        px4_autostart: int = 4001,
        px4_dir: str = "/workspace/src/PX4-Autopilot/",
        px4_simulator: str = "GZ",
        px4_model_name: Optional[str] = None,  
        px4_render_engine: Optional[str] = "ogre",
        standalone: bool = True,      
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
        gz_world=gz_world,
        px4_autostart=px4_autostart,
        px4_dir=px4_dir,
        px4_simulator=px4_simulator,
        px4_model_name=px4_model_name,
        px4_sim_model=drone_type,
        px4_gz_model_pose=spawn_position,
        px4_render_engine=px4_render_engine,
        standalone=standalone,
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
