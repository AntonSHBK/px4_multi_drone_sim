#!/usr/bin/env python3

from typing import Literal

import numpy as np

import rclpy
from rclpy.clock import Clock

from px4_msgs.msg import ( 
    TrajectorySetpoint, 
    VehicleStatus, 
    VehicleCommand,
    OffboardControlMode,
) 

from multi_drone.controllers.base.base_controller import BaseDroneController
from multi_drone.controllers.base.position_transformer import DroneLocalityState
from multi_drone.controllers.x500.states import (
    IdleState,
    ArmingState,
    TakeoffState,
    LoiterState,
    LandingState,
    DisarmState,
    OffboardState,
    DroneState)
from multi_drone_msg.msg import DroneInformMsg, DroneParamsMsg


class X500Params:
    nav_state: int = VehicleStatus.NAVIGATION_STATE_MAX
    arm_state: int = VehicleStatus.ARMING_STATE_ARMED
    
    offboard_mode: bool = False
    landing: bool = False 
    arm_message: bool = False
    
    flight_check: bool = False
    failsafe: bool = False
    
    def update_params(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flight_check = msg.pre_flight_checks_pass
        
    def to_msg(self) -> DroneParamsMsg:
        msg = DroneParamsMsg()
        msg.arm_message = self.arm_message
        msg.arm_state = self.arm_state
        msg.nav_state = self.nav_state
        msg.offboard_mode = self.offboard_mode
        msg.landing = self.landing
        msg.failsafe = self.failsafe
        msg.flight_check = self.flight_check
        return msg
    

class X500BaseController(BaseDroneController):
        
    def __init__(
        self,
        drone_id:int=1,
        drone_type:str='x500',
        default_position=[0.0,0.0,0.0],
        default_orientation=[0.0,0.0,0.0],
        timer_state_period=0.5,
        timer_inform_of_drone=0.5):
                                    
        super().__init__(drone_id, drone_type, default_position, default_orientation)       
        
        self.log_info(f"Инициализация дрона с ID {self.drone_id}, тип {self.drone_type}, позиция {self.default_world_position_ENU}")
        
        self.params = X500Params()
        
        self.states = {
            "IDLE": IdleState(self),
            "ARMING": ArmingState(self),
            "TAKEOFF": TakeoffState(self),
            "LOITER": LoiterState(self),
            "OFFBOARD": OffboardState(self),
            "LANDING": LandingState(self),
            "DISARM": DisarmState(self),
        }
        self.current_state: DroneState = self.states["IDLE"]
        
        self.offboard_commander = OffboardCommander(self)
        
        self.timer_state = self.create_timer(
            timer_state_period, self.update_state)
        
        self.timer_inform_of_drone = self.create_timer(
            timer_inform_of_drone, self.publish_inform_of_drone)
        
        self.subscriber_vehicle_status = self.create_subscription(
            VehicleStatus,
            f'{self.prefix_px}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            self.qos_profile_unreliable
        )
        
        self.publisher_inform_of_drone = self.create_publisher(
            DroneInformMsg,
            f'{self.prefix_name}/out/inform_of_drone',
            self.qos_profile_unreliable
        )        
    
    def vehicle_status_callback(self, msg: VehicleStatus):
        self.params.update_params(msg)

    def set_state(self, state_name: str):
        if state_name not in self.states:
            raise ValueError(f"Неизвестное состояние: {state_name}")
        self.current_state.exit()
        self.current_state = self.states[state_name]
        self.current_state.enter()

    def update_state(self):
        self.current_state.handle()
        
    def publish_inform_of_drone(self):
        msg = DroneInformMsg()
        msg.params = self.params.to_msg()
        msg.current_position = self.current_position.to_msg()
        msg.target_position = self.target_position.to_msg()
        self.publisher_inform_of_drone.publish(msg)

    def takeoff(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5.0)

    def enable_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                
    def disable_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 0.0)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
    
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        
    def take_off(self):        
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5.0)
    
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        
    def stop(self):
        pass
    
    def stop_on_offboard(self):
        pass
        
    def set_home_to_current_position(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_HOME, param1=1.0)
        

class OffboardCommander:
    """
    Класс для управления командами в режиме Offboard.
    """
    def __init__(self, controller: X500BaseController, timer_offboard=0.1):
        """
        Инициализация командера.

        :param controller: Ссылка на базовый контроллер дрона.
        """
        self._controller = controller
        self._active = False
        
        self.publisher_offboard_mode = controller.create_publisher(
            OffboardControlMode,
            f'{controller.prefix_px}/fmu/in/offboard_control_mode',
            controller.qos_profile_reliable
        )
        self.publisher_trajectory = controller.create_publisher(
            TrajectorySetpoint,
            f'{controller.prefix_px}/fmu/in/trajectory_setpoint',
            controller.qos_profile_reliable
        )
        
        self._position = np.array([-7.0, 2.0, -20.0])
        # self._position = np.array([np.nan, np.nan, np.nan])
        self._velocity = np.array([np.nan, np.nan, np.nan])
        self._acceleration = np.array([np.nan, np.nan, np.nan])
        self._yaw = np.nan
        self._yaw_speed = np.nan
        
        self.mode: Literal['position', 'velocity', 'mixed'] = 'mixed'

        self.command_timer = self._controller.create_timer(
            timer_offboard, self._timer_offboard_callback
        )
        
    def update(
        self, 
        position:np.ndarray=None, 
        velocity:np.ndarray=None, 
        acceleration:np.ndarray=None, 
        yaw:float=None, 
        yaw_speed:float=None,
        mode: Literal['position', 'velocity', 'mixed'] = None,
        system: Literal[
            "local_NED", "local_ENU", 'global_ENU', 'global_NED'
        ] = 'global_NED'
    ):
        """
        Обновляет параметры состояния объекта. Если параметр не указан, он будет установлен в np.nan.
        
        :param position: np.ndarray или список из 3 элементов [x, y, z], позиция.
        :param velocity: np.ndarray или список из 3 элементов [vx, vy, vz], скорость.
        :param acceleration: np.ndarray или список из 3 элементов [ax, ay, az], ускорение.
        :param yaw: float, ориентация в радианах.
        :param yaw_speed: float, скорость изменения yaw в радианах/сек.
        """
        
        if position:
            self._controller.target_position.update_position(position, system=system)
            self._position = self._controller.target_position.get_position(system=system)
        else:
            self._position = np.array([np.nan, np.nan, np.nan])
            
        if velocity:
            self._controller.target_position.update_velocity(velocity, system=system)
            self._velocity = self._controller.target_position.get_velocity(system=system)
        else:
            self._velocity = np.array([np.nan, np.nan, np.nan])
            
        if acceleration:
            pass
        
        if yaw:
            self._yaw = yaw
        else:
            self._yaw = np.nan
        
        if yaw_speed:
            self._yaw_speed = yaw_speed
        else:
            self._yaw_speed = np.nan
        
        if mode:
            self.mode = mode

    def send_offboard_mode(self, position=False, velocity=False, acceleration=False):
        """
        Отправляет команду для включения Offboard режима.
        """
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = position
        offboard_msg.velocity = velocity
        offboard_msg.acceleration = acceleration
        self.publisher_offboard_mode.publish(offboard_msg)

    def activate(self):
        self._active = True
        
    def desactivate(self):
        self._active = False
    
    def send_trajectory_setpoint(self):
        """
        Отправляет траекторную точку в режиме Offboard.
        """
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        # TODO: возможно стоить убрать проверку на nan
        
        trajectory_msg.position[:3] = self._position
        
        if not np.isnan(self._velocity).all():
            trajectory_msg.velocity[:3] = self._velocity
        if not np.isnan(self._acceleration).all():
            trajectory_msg.acceleration[:3] = self._acceleration
        
        trajectory_msg.yaw = self._yaw
        
        if not np.isnan(self._yaw_speed):
            trajectory_msg.yawspeed = self._yaw_speed

        self.publisher_trajectory.publish(trajectory_msg)
    
    def _timer_offboard_callback(self):
        """
        Периодически отправляет команды в режиме Offboard.
        """
        if not self._active:
            return
        
        if self.mode == "mixed":
            self.send_offboard_mode(position=True, velocity=True)
            self.send_trajectory_setpoint()
        elif self.mode == "position":
            self.send_offboard_mode(position=True)
            self.send_trajectory_setpoint()
        elif self.mode == "velocity":
            self.send_offboard_mode(velocity=True)
            self.send_trajectory_setpoint()        


def main():    
    rclpy.init()     
    control_node = X500BaseController()
    
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        control_node.get_logger().info("Остановка контроллера.")
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        
        
        
        