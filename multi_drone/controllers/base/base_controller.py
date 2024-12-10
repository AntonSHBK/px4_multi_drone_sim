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

from multi_drone.controllers.base.base_data import (
    PositionData,
    OrientationData,
    VelocityData,
)

from multi_drone_msg.msg import LocalAndGlobalCoordinatesMsg

class DroneLocalityState():
    # NED - СК дрона
    # ENU - СК мира
    def __init__(
        self,
        world_position = PositionData(x=0, y=0, z=0),
        world_orientation = OrientationData(roll=0, pitch=0, yaw=0)
    ):
        self.position_local_ENU = PositionData()
        self.velocity_local_ENU = VelocityData()
        # self.orientation_local_ENU = OrientationData()
        
        self.position_local_NED = PositionData()
        self.velocity_local_NED = VelocityData()
        # self.orientation_local_NED = OrientationData()
        
        self.position_global_ENU = PositionData()
        self.velocity_global_ENU = VelocityData()
        # self.orientation_global_ENU = OrientationData()
        
        self.position_global_NED = PositionData()
        self.velocity_global_NED = VelocityData()
        # self.orientation_global_NED = OrientationData()
        
        self.world_position_ENU = world_position
        self.world_orientation_ENU = world_orientation  
        
        self.yaw_NED = 0.0
        self.yaw_ENU = 0.0 
        
    def get_position(self):
        pass
    
    def get_velocity(self):
        pass
    
    def get_orientation(self):
        pass

    def update_position(self, array_p:np.ndarray):  
        
        self.position_local_NED.update_from_array(array_p)
        
        self.position_local_ENU.update_from_array(
            self.position_local_NED.to_ENU())
              
        self.position_global_ENU.update_from_array(
            self.position_local_ENU.to_global(
                reference_position=self.world_position_ENU.to_array(),
                reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
        
        self.position_global_NED.update_from_array(
            self.position_global_ENU.to_NED())
    
    def update_velocity(self, array_v:np.ndarray):
        
        self.velocity_local_NED.update_from_array(array_v)
        
        self.velocity_local_ENU.update_from_array(
            self.velocity_local_NED.to_ENU())
        
        self.velocity_global_ENU.update_from_array(
            self.velocity_local_ENU.to_global(
                reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
        
        self.velocity_global_NED.update_from_array(
            self.velocity_global_ENU.to_NED())
        
    def update_orientation(self, array_q:np.ndarray):
        
        # Пока не уверен что нужно и нужно проверить расчёт
        
        # self.orientation_local_NED.update_from_quaternion_array(array_q)
        
        # self.orientation_local_ENU.update_from_quaternion_array(
        #     self.orientation_local_NED.quaternion.to_ENU())
        
        # self.orientation_global_ENU.update_from_quaternion_array(            
        #     self.orientation_local_ENU.quaternion.to_global(
        #         self.world_orientation_ENU.quaternion.to_array()))
        
        # self.orientation_global_NED.update_from_quaternion_array(
        #     self.orientation_global_ENU.quaternion.to_NED())
        
        self.yaw_ENU = float(np.arctan2(2.0 * (array_q[3] * array_q[2] + array_q[0] * array_q[1]), 
                                  1.0 - 2.0 * (array_q[1] * array_q[1] + array_q[2] * array_q[2])))        
        self.yaw_NED = -self.yaw_ENU
        
    def to_msg(self) -> LocalAndGlobalCoordinatesMsg:
        msg = LocalAndGlobalCoordinatesMsg()
        msg.local_enu.position = self.position_local_ENU.to_vector3()
        msg.local_enu.velocity = self.velocity_local_ENU.to_vector3()
        msg.local_enu.yaw = self.yaw_ENU
        
        msg.local_ned.position = self.position_local_NED.to_vector3()
        msg.local_ned.velocity = self.velocity_local_NED.to_vector3()
        msg.local_ned.yaw = self.yaw_NED
        
        msg.global_enu.position = self.position_global_ENU.to_vector3()
        msg.global_enu.velocity = self.velocity_global_ENU.to_vector3()
        msg.global_enu.yaw = self.yaw_ENU
        
        msg.global_ned.position = self.position_global_NED.to_vector3()
        msg.global_ned.velocity = self.velocity_global_NED.to_vector3()
        msg.global_ned.yaw = self.yaw_NED
        
        return msg              


class BaseDroneController(Node):
    def __init__(
        self,
        drone_id: int,
        drone_type: str,
        default_position=[0.0,0.0,0.0],  # Позиция задаётся списком [x, y, z] метры
        default_orientation=[0.0,0.0,0.0],  # Ориентация задаётся списком [roll, pitch, yaw] радианы
    ):        
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
                
        self.drone_id = drone_id
        self.drone_type = drone_type
        
        # Префиксы для топиков
        self.prefix_px = f"px4_{self.drone_id}"
        self.prefix_name = f"id_{self.drone_id}_{self.drone_type}"
        
        super().__init__(f'{self.prefix_name}_controller')
        
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
        self.current_position.update_position(np.array([
            msg.x, msg.y, msg.z
        ]))
        self.current_position.update_velocity(np.array([
            msg.vx, msg.vy, msg.vz
        ]))
    
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
        
      
def main(args=None):
    rclpy.init(args=args)    
    control_node = BaseDroneController(1, 'x500')
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()