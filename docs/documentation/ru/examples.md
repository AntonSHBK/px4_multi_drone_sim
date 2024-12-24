### examples.md

#### **Примеры использования пакета**

Этот документ содержит примеры использования различных компонентов пакета для управления дронами. Эти примеры охватывают базовые сценарии, такие как загрузка Gazebo и настройка симуляции, выполнение команд движения и управление дронами.

---

### **1. Запуск симуляции Gazebo**

В этом примере показано, как запустить симуляцию Gazebo с использованием пользовательских моделей и миров. Основной скрипт настраивает путь к моделям и миру, а затем запускает Gazebo.

#### **Код:**
```python
#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare

from multi_drone.utils.methods import load_yaml_params
from multi_drone.scripts.gazebo_server import run_simulation_gazebo


def generate_launch_description():
    
    # Определение пути к файлу параметров
    pkg_share = FindPackageShare('custom_models').find('custom_models')
    params_file = os.path.join(pkg_share, 'config', 'initial_params.yml')

    # Загрузка параметров из YAML файла
    params: dict = load_yaml_params(params_file)
    default_params: dict = params.get('default_params', {})

    world_file = default_params.get('world_file', 'default')
    github_repo = default_params.get('github_repo', '~/PX4-gazebo-models')
    
    # Запуск Gazebo с параметрами
    gazebo_server = run_simulation_gazebo(
        world=world_file,
        px4_model_store=github_repo,
        custom_model_store=pkg_share,
        custom_model_store_other=[],
        headless=False,
        overwrite=False,
        gz_ip=None,
        gz_partition=None
    )

    return LaunchDescription([gazebo_server])

if __name__ == "__main__":
    generate_launch_description()
```

#### **Пояснение:**
1. **Загрузка параметров из YAML:**
   - Параметры симуляции, такие как файл мира (`world_file`) и репозиторий моделей (`github_repo`), загружаются из файла `initial_params.yml`.
2. **Настройка пути к моделям:**
   - Модели загружаются из пользовательской директории (`custom_model_store`) и стандартного репозитория PX4.
3. **Запуск симуляции:**
   - Используется функция `run_simulation_gazebo`, которая запускает Gazebo с заданными параметрами.

---

### **2. Запуск дронов с кастомным контроллером**

В этом разделе показано, как запустить несколько дронов с использованием кастомного контроллера. Кастомный контроллер расширяет базовый контроллер `X500Controller`, добавляя поддержку пользовательских команд.

---

#### **Код:**

```python
#!/usr/bin/env python

import os
import logging

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

from multi_drone.utils.methods import load_yaml_params
from multi_drone.scripts.runner import launch_robot, get_microxrce_agent_exec

def generate_launch_description():
    
    pkg_multirobots = get_package_share_directory('multirobots')
    params_file = os.path.join(pkg_multirobots, 'config', 'drone_config.yml')
    robots_config = load_yaml_params(params_file)

    launch_descriptions = []

    # Добавление ноды для запуска MicroXRCEAgent
    microxrce_agent = get_microxrce_agent_exec(udp='udp4', port='8888')
    launch_descriptions.append(microxrce_agent)

    # Цикл по каждому роботу в конфигурации
    for robot in robots_config['robots']:
        
        logging.info(f"Запускаем дрон {robot['drone_id']} с типом {robot['drone_type']}")

        launch_descriptions.extend(
            launch_robot(
                drone_id=robot['drone_id'],
                drone_type=robot['drone_type'],
                spawn_position=robot['position'],
                px4_autostart=robot['px4_autostart'],
                px4_dir="/workspace/src/PX4-Autopilot/",
                terminal='bash',
                package_name=robot['package_name'],
                controller_script=robot['controller_script'],
                additional_params={}
            )
        )

    return LaunchDescription(launch_descriptions)

if __name__ == '__main__':
    generate_launch_description()
```

---

#### **Пояснение:**

1. **Файл конфигурации дронов:**
   - Файл `drone_config.yml` описывает параметры каждого дрона:
     ```yaml
     robots:
       - drone_id: 1
         drone_type: x500
         position: [0, 0, 0, 0, 0, 0]
         px4_autostart: 4001
         package_name: multirobots
         controller_script: custom_x500.py
       - drone_id: 2
         drone_type: x500
         position: [5, 5, 0, 0, 0, 0]
         px4_autostart: 4002
         package_name: multirobots
         controller_script: custom_x500.py
     ```

2. **Запуск MicroXRCEAgent:**
   - Этот компонент обеспечивает связь между PX4 и ROS 2.

3. **Запуск дронов:**
   - Для каждого дрона в конфигурации вызывается функция `launch_robot`, которая:
     - Запускает PX4.
     - Запускает ROS 2 ноду с кастомным контроллером.

---

#### **Создание кастомного контроллера**

Кастомный контроллер `CustomX500Controller` наследует `X500Controller` и добавляет пользовательскую команду `G101`.

```python
#!/usr/bin/env python3

import rclpy
from multi_drone.controllers.x500.x500 import X500Controller
from multirobots.scripts.move_commands.x500.example import TestCustomCommand

class CustomX500Controller(X500Controller):
        
    def __init__(
        self,
        drone_id:int=1,
        drone_type:str='x500',
        default_position=[0.0,0.0,0.0],
        default_orientation=[0.0,0.0,0.0]):  
                                          
        super().__init__(
            drone_id=drone_id, 
            drone_type=drone_type, 
            default_position=default_position,
            default_orientation=default_orientation           
        )   
        
        self.g_code_commander.add_command_class('G101', TestCustomCommand)     
       
def main():    
    rclpy.init()     
    control_node = CustomX500Controller()
    
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        control_node.get_logger().info("Остановка контроллера.")
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

#### **Создание кастомной команды**

Пользовательская команда `G101` описывает движение дрона, например, по спирали.

```python
from multi_drone.move_commands.x500.g_code.base_move_command import BaseMoveGCommand

class TestCustomCommand(BaseMoveGCommand):
    """
    Команда для движения дрона по спирали.
    """
    def __init__(self, params):
        super().__init__("G101")
        self.params = params

    def update_targets_positions(self):
        # Пример логики формирования целевых точек для спирального движения
        pass

    def to_dict(self) -> dict:
        return {"name": self.name, "params": self.params}

    @classmethod
    def from_dict(cls, data: dict):
        return cls(params=data.get("params"))

    def __repr__(self):
        return f"TestCustomCommand(params={self.params})"
```

---

#### **Запуск**

1. **Файл конфигурации дронов:**
   Убедитесь, что в `drone_config.yml` указан кастомный контроллер (`custom_x500.py`).

2. **Команда запуска:**
   ```bash
   ros2 launch multirobots launch_drones.launch.py
   ```