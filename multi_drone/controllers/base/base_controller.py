#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, 
    QoSReliabilityPolicy, 
    QoSHistoryPolicy, 
    QoSDurabilityPolicy
)

from px4_msgs.msg import (
    VehicleAttitude, 
    VehicleCommand,
    VehicleLocalPosition
)

from multi_drone.controllers.base.base_data import PositionData, OrientationData
from multi_drone.controllers.base.position_transformer import DroneLocalityState


class BaseDroneController(Node):
    def __init__(
        self,
        drone_type: str = 'x500',
        drone_id: int = 1,        
        default_position: list = [0.0,0.0,0.0],  # Позиция задаётся списком [x, y, z] метры
        default_orientation: list = [0.0,0.0,0.0],  # Ориентация задаётся списком [roll, pitch, yaw] радианы
    ):        
        super().__init__(f'drone_controller')
        from rcl_interfaces.msg import ParameterDescriptor
        self.drone_type = drone_type
        self.drone_id = drone_id
        
        self.declare_parameter('drone_id', value=drone_id, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('drone_type', value=drone_type, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('default_position', value=default_position, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('default_orientation', value=default_orientation, descriptor=ParameterDescriptor(dynamic_typing=True))

        self.drone_id: int = self.get_parameter('drone_id').value
        self.drone_type: str = self.get_parameter('drone_type').value        
        default_position: list = self.get_parameter('default_position').value
        default_orientation: list = self.get_parameter('default_orientation').value
        
        self.default_world_position_ENU = PositionData(
            x=default_position[0],
            y=default_position[1],
            z=default_position[2]
        )
        self.default_world_orientation_ENU = OrientationData(
            roll=default_orientation[0],
            pitch=default_orientation[1],
            yaw=default_orientation[2]
        )
        
        self.current_position = DroneLocalityState(
            self.default_world_position_ENU,
            self.default_world_orientation_ENU
        )
        self.target_position = DroneLocalityState(
            self.default_world_position_ENU,
            self.default_world_orientation_ENU
        )
        
        # Префиксы для топиков
        self.prefix_px = f"px4_{self.drone_id}"
        self.prefix_name = f"id_{self.drone_id}_{self.drone_type}"
        
        self.qos_profile_unreliable = self.get_qos_profile(
            reliable=False, depth=5
        )
        self.qos_profile_reliable = self.get_qos_profile(
            reliable=True, depth=10
        )  
             
        # self.delta_position = Vector3(x=0.0, y=0.0, z=0.0)
        # self.delta_orientation = Vector3(x=0.0, y=0.0, z=0.0)  
                   
        self.subscriber_vehicle_attitude = self.create_subscription(
            VehicleAttitude,
            f'{self.prefix_px}/fmu/out/vehicle_attitude',
            self.callback_vehicle_attitude,
            self.qos_profile_unreliable
        )
        
        self.subscriber_local_position = self.create_subscription(
            VehicleLocalPosition,
            f'{self.prefix_px}/fmu/out/vehicle_local_position',
            self.callback_local_position,
            self.qos_profile_unreliable
        ) 
        
        # self.global_position_sub = self.create_subscription(
        #     VehicleLocalPosition,
        #     f'{self.prefix_px}/fmu/out/vehicle_local_position',
        #     self.callback_local_position,
        #     self.qos_profile_unreliable
        # )        
                
        self.publisher_vehicle_command = self.create_publisher(
            VehicleCommand,
            f'{self.prefix_px}/fmu/in/vehicle_command',
            self.qos_profile_reliable
        )
        
    def get_qos_profile(self, reliable=True, depth=10):
        """
        Возвращает QoS-профиль. По умолчанию надёжный (RELIABLE).
        """
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE
            if reliable
            else QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth,
        )
    
    def callback_vehicle_attitude(self, msg: VehicleAttitude):
        """
        Обработчик для получения ориентации дрона в системе ENU и преобразования в NED.
        """
        self.current_position.update_orientation(msg.q)
            
    def callback_local_position(self, msg: VehicleLocalPosition):
        """
        Получение локальных координат дрона обработка скоростей
        в системах координат NED и ENU.
        """
        self.current_position.update_position(
            np.array([msg.x, msg.y, msg.z]), 
            system='local_NED')
        self.current_position.update_velocity(
            np.array([msg.vx, msg.vy, msg.vz]), 
            system='local_NED')
    
    def publish_vehicle_command(
        self, 
        command, 
        param1=0.0, 
        param2=0.0, 
        param3=0.0, 
        param4=0.0, 
        param5=0.0, 
        param6=0.0, 
        param7=5.0, 
        target_system=None, 
        target_component=1, 
        source_system=None, 
        source_component=1, 
        from_external=True
    ):
        """
        Публикует команду VehicleCommand для управления дроном.

        Параметры:
        ----------
        command : int
            Идентификатор команды (например, армирование, взлёт, посадка и т.д.).
        param1 : float, optional
            Первичный параметр команды. Зависит от типа команды (по умолчанию 0.0).
            Например:
            - Для армирования: 1.0 (ARM), 0.0 (DISARM).
            - Для взлёта: время в секундах.
        param2 : float, optional
            Вторичный параметр команды. Используется для дополнительных данных (по умолчанию 0.0).
            Например:
            - Для SET_MODE: режим полёта (1: AUTO, 6: OFFBOARD и т.д.).
        param3 : float, optional
            Третичный параметр команды (по умолчанию 0.0).
        param4 : float, optional
            Четвёртый параметр команды (по умолчанию 0.0).
        param5 : float, optional
            Пятый параметр команды (по умолчанию 0.0). Может использоваться для координаты X.
        param6 : float, optional
            Шестой параметр команды (по умолчанию 0.0). Может использоваться для координаты Y.
        param7 : float, optional
            Седьмой параметр команды (по умолчанию 5.0). Часто используется для высоты (координаты Z).
        target_system : int, optional
            Система, которая должна выполнить команду. Если не указано, используется `self.drone_id + 1`.
        target_component : int, optional
            Компонент, который должен выполнить команду (по умолчанию 1).
        source_system : int, optional
            Система, отправляющая команду. Если не указано, используется `self.drone_id`.
        source_component : int, optional
            Компонент, отправляющий команду (по умолчанию 1).
        from_external : bool, optional
            Флаг, показывающий, что команда отправлена извне (по умолчанию True).
        """
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command
        msg.target_system = target_system if target_system is not None else self.drone_id + 1
        msg.target_component = target_component
        msg.source_system = source_system if source_system is not None else self.drone_id
        msg.source_component = source_component
        msg.from_external = from_external
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(msg)


    def log_error(self, message):
        self.get_logger().error(message)

    def log_info(self, message):
        self.get_logger().info(message)
        
      
def main():  
    rclpy.init()     
    control_node = BaseDroneController()
    
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        control_node.get_logger().info("Остановка контроллера.")
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()