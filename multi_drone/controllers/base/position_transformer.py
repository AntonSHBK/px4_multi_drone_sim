from typing import Union, Literal

import numpy as np

from multi_drone.controllers.base.base_data import (
    PositionData,
    OrientationData,
    VelocityData,
)

from multi_drone_msg.msg import LocalAndGlobalCoordinatesMsg

class DroneLocalityState():
    """
    Класс для управления состоянием дрона в различных системах координат (NED и ENU).

    Атрибуты:
    ----------
    - position_local_ENU: Позиция в локальной системе ENU.
    - velocity_local_ENU: Скорость в локальной системе ENU.
    - position_local_NED: Позиция в локальной системе NED.
    - velocity_local_NED: Скорость в локальной системе NED.
    - position_global_ENU: Позиция в глобальной системе ENU.
    - velocity_global_ENU: Скорость в глобальной системе ENU.
    - position_global_NED: Позиция в глобальной системе NED.
    - velocity_global_NED: Скорость в глобальной системе NED.
    - world_position_ENU: Позиция мировых координат в ENU.
    - world_orientation_ENU: Ориентация мировых координат в ENU.
    - yaw_NED: Угол рыскания (yaw) в системе NED.
    - yaw_ENU: Угол рыскания (yaw) в системе ENU.
    """
    
    def __init__(
        self,
        world_position = PositionData(x=0, y=0, z=0),
        world_orientation = OrientationData(roll=0, pitch=0, yaw=0)
    ):
        """
        Инициализирует состояние дрона с заданной позицией и ориентацией.

        Параметры:
        ----------
        - world_position (PositionData): Начальная позиция в глобальной системе ENU.
        - world_orientation (OrientationData): Начальная ориентация в глобальной системе ENU.
        """
        
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
        
    def get_position(
        self, 
        system: Literal[
            "local_NED", "local_ENU", 'global_ENU', 'global_NED'
        ] = 'local_NED'
    ) -> np.ndarray:
        
        if system == 'local_NED':
            return self.position_local_NED.to_array()
        elif system == 'local_ENU': 
            return self.position_local_ENU.to_array()
        elif system == 'global_ENU':
            return self.position_global_ENU.to_array()
        elif system == 'global_NED':
            return self.position_global_NED.to_array()
        else:
            raise ValueError(f"Unknown system: {system}")
    
    def get_velocity(
        self, 
        system: Literal[
            "local_NED", "local_ENU", 'global_ENU', 'global_NED'
        ] = 'local_NED'
    ) -> np.ndarray:
        
        if system == 'local_NED':
            return self.velocity_local_NED.to_array()
        elif system == 'local_ENU': 
            return self.velocity_local_ENU.to_array()
        elif system == 'global_ENU':
            return self.velocity_global_ENU.to_array()
        elif system == 'global_NED':
            return self.velocity_global_NED.to_array()
        else:
            raise ValueError(f"Unknown system: {system}")
    
    def get_orientation(
        self,
        system: Literal[
            "local_NED", "local_ENU", "global_ENU", "global_NED"
        ] = "local_NED",
    ) -> float:
        """
        Возвращает угол рыскания (yaw) в указанной системе координат.

        Параметры:
        ----------
        - system (Literal): Система координат ("local_NED", "local_ENU", "global_ENU", "global_NED").

        Возвращает:
        ----------
        - float: Угол yaw в радианах.
        """
        if system == "local_NED":
            return self.yaw_NED
        elif system == "local_ENU":
            return self.yaw_ENU
        elif system == "global_ENU":
            return self.yaw_ENU
        elif system == "global_NED":
            return self.yaw_NED
        else:
            raise ValueError(f"Unknown system: {system}")


    def update_position(
            self, 
            array_p: np.ndarray, 
            system: Literal[
                    "local_NED", "local_ENU", 'global_ENU', 'global_NED'
                ] = 'local_NED'
        ):  
        if system == 'local_NED':
            self.position_local_NED.update_from_array(array_p)
            
            self.position_local_ENU.update_from_array(
                self.position_local_NED.to_ENU())
                
            self.position_global_ENU.update_from_array(
                self.position_local_ENU.to_global(
                    reference_position=self.world_position_ENU.to_array(),
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.position_global_NED.update_from_array(
                self.position_global_ENU.to_NED())
            
        elif system == 'local_ENU':
            self.position_local_ENU.update_from_array(array_p)
            
            self.position_local_NED.update_from_array(
                self.position_local_ENU.to_NED())
                
            self.position_global_ENU.update_from_array(
                self.position_local_ENU.to_global(
                    reference_position=self.world_position_ENU.to_array(),
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.position_global_NED.update_from_array(
                self.position_global_ENU.to_NED())
            
        elif system == 'global_ENU':
            self.position_global_ENU.update_from_array(array_p)
            
            self.position_local_ENU.update_from_array(
                self.position_global_ENU.to_local(
                    reference_position=self.world_position_ENU.to_array(),
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.position_local_NED.update_from_array(
                self.position_local_ENU.to_NED())
            
            self.position_global_NED.update_from_array(
                self.position_global_ENU.to_NED())
            
        elif system == 'global_NED':
            self.position_global_NED.update_from_array(array_p)
            
            self.position_global_ENU.update_from_array(
                self.position_global_NED.to_ENU())
                
            self.position_local_ENU.update_from_array(
                self.position_global_ENU.to_local(
                    reference_position=self.world_position_ENU.to_array(),
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.position_local_NED.update_from_array(
                self.position_local_ENU.to_NED())
        else:
            raise ValueError(f"Unknown system: {system}")

    def update_velocity(
        self, 
        array_v: np.ndarray, 
        system: Literal[
                "local_NED", "local_ENU", 'global_ENU', 'global_NED'
            ] = 'local_NED'
    ):
        if system == 'local_NED':
            self.velocity_local_NED.update_from_array(array_v)
            
            self.velocity_local_ENU.update_from_array(
                self.velocity_local_NED.to_ENU())
            
            self.velocity_global_ENU.update_from_array(
                self.velocity_local_ENU.to_global(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.velocity_global_NED.update_from_array(
                self.velocity_global_ENU.to_NED())
        elif system == 'local_ENU':
            self.velocity_local_ENU.update_from_array(array_v)
            
            self.velocity_local_NED.update_from_array(
                self.velocity_local_ENU.to_NED())
            
            self.velocity_global_ENU.update_from_array(
                self.velocity_local_ENU.to_global(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.velocity_global_NED.update_from_array(
                self.velocity_global_ENU.to_NED())
        elif system == 'global_ENU':
            self.velocity_global_ENU.update_from_array(array_v)
            
            self.velocity_local_ENU.update_from_array(
                self.velocity_global_ENU.to_local(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.velocity_local_NED.update_from_array(
                self.velocity_local_ENU.to_NED())
            
            self.velocity_global_NED.update_from_array(
                self.velocity_global_ENU.to_NED())
        elif system == 'global_NED':
            self.velocity_global_NED.update_from_array(array_v)
            
            self.velocity_global_ENU.update_from_array(
                self.velocity_global_NED.to_ENU())
            
            self.velocity_local_ENU.update_from_array(
                self.velocity_global_ENU.to_local(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.velocity_local_NED.update_from_array(
                self.velocity_local_ENU.to_NED())
        else:
            raise ValueError(f"Unknown system: {system}")
        
    def update_orientation(
        self, 
        array_q: np.ndarray, 
        system: Literal[
                "local_NED", "local_ENU", 'global_ENU', 'global_NED'
            ] = 'local_NED'
    ):
        
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