#!/usr/bin/env python3

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

import rclpy
from rclpy.clock import Clock

from px4_msgs.msg import (
    OffboardControlMode, 
    TrajectorySetpoint, 
    VehicleStatus, 
    VehicleCommand,
)

from multi_drone.controllers.base.base_controller import BaseDroneController
from multi_drone.controllers.x500.states import*

from multi_drone_msg.msg import RobotState


@dataclass
class X500Params:
    nav_state: int = VehicleStatus.NAVIGATION_STATE_MAX
    arm_state: int = VehicleStatus.ARMING_STATE_ARMED
    offboard_mode: bool = False
    landing: bool = False
    flightCheck: bool = False
    arm_message: bool = False
    failsafe: bool = False
    current_state: str = "IDLE"
    
    def update_state(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass


class X500BaseController(BaseDroneController):
        
    def __init__(
        self,
        drone_id: int,
        default_position=[0.0,0.0,0.0],
        default_orientation=[0.0,0.0,0.0]):
                            
        super().__init__(drone_id, 'x500', default_position, default_orientation)       
        
        self.log_info(f"Инициализация дрона с ID {self.drone_id}, тип {self.drone_type}, позиция {self.default_world_position_ENU}")

        self._init_params()
        self._init_subscribers()
        self._init_publisher()           
        self.set_home_to_current_position()        
        
    def _init_params(self):
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
        
    def _init_timers(self):
        timer_state_period = 0.5
        self.timer_state = self.create_timer(
            timer_state_period, self.update_state)
        
    def _init_subscribers(self):
        
        self.subscriber_vehicle_status = self.create_subscription(
            VehicleStatus,
            f'{self.prefix_px}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            self.qos_profile_unreliable
        )
        
    def _init_publisher(self):  
        
        self.publisher_state_of_robot = self.create_publisher(
            RobotState,
            f'{self.prefix_name}/out/state_of_robot',
            self.qos_profile_unreliable
        )        
    
    def vehicle_status_callback(self, msg: VehicleStatus):
        self.params.update_state(msg)

    def offboard_cmd_callback(self, msg: TrajectorySetpoint):

        if not any(np.isnan([msg.position[0], msg.position[1], msg.position[2]])):
                        
            self.next_position_global_ENU.x = msg.position[0].item()
            self.next_position_global_ENU.y = msg.position[1].item()
            self.next_position_global_ENU.z = msg.position[2].item()
            
            position_global_NED_x, position_global_NED_y, position_global_NED_z = enu_to_ned(
                msg.position[0].item(),                
                msg.position[1].item(), 
                msg.position[2].item())
                        
            self.next_position_global_NED.x = position_global_NED_x
            self.next_position_global_NED.y = position_global_NED_y
            self.next_position_global_NED.z = position_global_NED_z
            
            self.next_position_local_ENU = global_to_local(
                self.next_position_global_ENU,
                self.default_world_position_ENU,
                self.default_world_orientation_ENU
            )
                
            position_local_NED_x, position_local_NED_y, position_local_NED_z = enu_to_ned(
                self.next_position_local_ENU.x,                
                self.next_position_local_ENU.y, 
                self.next_position_local_ENU.z)
                        
            self.next_position_local_NED.x = position_local_NED_x
            self.next_position_local_NED.y = position_local_NED_y
            self.next_position_local_NED.z = position_local_NED_z            
            
            # self.get_logger().info(f"Получены координаты FLU: ({self.next_position_global.x}, {self.next_position_global.y}, {self.next_position_global.z})")
        else:
            # Если координаты не заданы, устанавливаем NaN для всех позиций
            self.next_position_global_ENU.x = float('nan')
            self.next_position_global_ENU.y = float('nan')
            self.next_position_global_ENU.z = float('nan')

            self.next_position_global_NED.x = float('nan')
            self.next_position_global_NED.y = float('nan')
            self.next_position_global_NED.z = float('nan')

            self.next_position_local_ENU.x = float('nan')
            self.next_position_local_ENU.y = float('nan')
            self.next_position_local_ENU.z = float('nan')

            self.next_position_local_NED.x = float('nan')
            self.next_position_local_NED.y = float('nan')
            self.next_position_local_NED.z = float('nan')

        if not any(np.isnan([msg.velocity[0], msg.velocity[1], msg.velocity[2]])):
            self.next_velocity_global_ENU.x = msg.velocity[0].item()
            self.next_velocity_global_ENU.y = msg.velocity[1].item()
            self.next_velocity_global_ENU.z = msg.velocity[2].item()

            velocity_global_NED_x, velocity_global_NED_y, velocity_global_NED_z = enu_to_ned(
                msg.velocity[0].item(),
                msg.velocity[1].item(),
                msg.velocity[2].item()
            )

            self.next_velocity_global_NED.x = velocity_global_NED_x
            self.next_velocity_global_NED.y = velocity_global_NED_y
            self.next_velocity_global_NED.z = velocity_global_NED_z

            self.next_velocity_local_ENU = global_to_local(
                self.next_velocity_global_ENU,
                Vector3(x=0.0, y=0.0, z=0.0),  # Для скоростей сдвиг не применяется
                self.default_world_orientation_ENU
            )

            velocity_local_NED_x, velocity_local_NED_y, velocity_local_NED_z = enu_to_ned(
                self.next_velocity_local_ENU.x,
                self.next_velocity_local_ENU.y,
                self.next_velocity_local_ENU.z
            )

            self.next_velocity_local_NED.x = velocity_local_NED_x
            self.next_velocity_local_NED.y = velocity_local_NED_y
            self.next_velocity_local_NED.z = velocity_local_NED_z

        else:
            # Если скорости не заданы, устанавливаем NaN для всех значений скорости
            self.next_velocity_global_ENU.x = float('nan')
            self.next_velocity_global_ENU.y = float('nan')
            self.next_velocity_global_ENU.z = float('nan')

            self.next_velocity_global_NED.x = float('nan')
            self.next_velocity_global_NED.y = float('nan')
            self.next_velocity_global_NED.z = float('nan')

            self.next_velocity_local_ENU.x = float('nan')
            self.next_velocity_local_ENU.y = float('nan')
            self.next_velocity_local_ENU.z = float('nan')

            self.next_velocity_local_NED.x = float('nan')
            self.next_velocity_local_NED.y = float('nan')
            self.next_velocity_local_NED.z = float('nan')

        # Проверяем, передаётся ли yaw (ориентация)
        if not np.isnan(msg.yaw):
            self.next_yaw_ENU = msg.yaw.item()
            self.next_yaw_NED = -self.next_yaw_ENU
            # self.get_logger().info(f"Получен yaw ENU: {self.next_yaw_ENU}, yaw NED: {self.next_yaw_NED}")
        else:
            self.next_yaw_ENU = calculate_yaw_towards_target(self.current_position_local_ENU, self.next_position_local_ENU)
            self.next_yaw_NED = -self.next_yaw_ENU
            # self.get_logger().info(f"Рассчитан yaw ENU: {self.next_yaw_ENU}, yaw NED: {self.next_yaw_NED}")

        # self.get_logger().info(f"Преобразовано в координатную систему FLU: скорость=({self.next_velocity.x}, {self.next_velocity.y}, {self.next_velocity.z}), roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}")
        
    def timer_robot_of_state_callback(self):
        """
        Callback-функция для публикации состояния робота.
        """
        # Создаем сообщение состояния
        state_msg = RobotState()

        # Устанавливаем навигационное состояние и состояние арминга
        state_msg.nav_state = self.nav_state
        state_msg.arm_state = self.arm_state

        # Локальные координаты ENU
        state_msg.next_position_local_enu.x = self.next_position_local_ENU.x
        state_msg.next_position_local_enu.y = self.next_position_local_ENU.y
        state_msg.next_position_local_enu.z = self.next_position_local_ENU.z

        state_msg.next_velocity_local_enu.x = self.next_velocity_local_ENU.x
        state_msg.next_velocity_local_enu.y = self.next_velocity_local_ENU.y
        state_msg.next_velocity_local_enu.z = self.next_velocity_local_ENU.z

        state_msg.current_position_local_enu.x = self.current_position_local_ENU.x
        state_msg.current_position_local_enu.y = self.current_position_local_ENU.y
        state_msg.current_position_local_enu.z = self.current_position_local_ENU.z

        state_msg.current_velocity_local_enu.x = self.current_velocity_local_ENU.x
        state_msg.current_velocity_local_enu.y = self.current_velocity_local_ENU.y
        state_msg.current_velocity_local_enu.z = self.current_velocity_local_ENU.z

        # Локальные координаты NED
        state_msg.next_position_local_ned.x = self.next_position_local_NED.x
        state_msg.next_position_local_ned.y = self.next_position_local_NED.y
        state_msg.next_position_local_ned.z = self.next_position_local_NED.z

        state_msg.next_velocity_local_ned.x = self.next_velocity_local_NED.x
        state_msg.next_velocity_local_ned.y = self.next_velocity_local_NED.y
        state_msg.next_velocity_local_ned.z = self.next_velocity_local_NED.z

        state_msg.current_position_local_ned.x = self.current_position_local_NED.x
        state_msg.current_position_local_ned.y = self.current_position_local_NED.y
        state_msg.current_position_local_ned.z = self.current_position_local_NED.z

        state_msg.current_velocity_local_ned.x = self.current_velocity_local_NED.x
        state_msg.current_velocity_local_ned.y = self.current_velocity_local_NED.y
        state_msg.current_velocity_local_ned.z = self.current_velocity_local_NED.z

        # Глобальные координаты ENU
        state_msg.next_position_global_enu.x = self.next_position_global_ENU.x
        state_msg.next_position_global_enu.y = self.next_position_global_ENU.y
        state_msg.next_position_global_enu.z = self.next_position_global_ENU.z

        state_msg.next_velocity_global_enu.x = self.next_velocity_global_ENU.x
        state_msg.next_velocity_global_enu.y = self.next_velocity_global_ENU.y
        state_msg.next_velocity_global_enu.z = self.next_velocity_global_ENU.z

        state_msg.current_position_global_enu.x = self.current_position_global_ENU.x
        state_msg.current_position_global_enu.y = self.current_position_global_ENU.y
        state_msg.current_position_global_enu.z = self.current_position_global_ENU.z

        state_msg.current_velocity_global_enu.x = self.current_velocity_global_ENU.x
        state_msg.current_velocity_global_enu.y = self.current_velocity_global_ENU.y
        state_msg.current_velocity_global_enu.z = self.current_velocity_global_ENU.z

        # Глобальные координаты NED
        state_msg.next_position_global_ned.x = self.next_position_global_NED.x
        state_msg.next_position_global_ned.y = self.next_position_global_NED.y
        state_msg.next_position_global_ned.z = self.next_position_global_NED.z

        state_msg.next_velocity_global_ned.x = self.next_velocity_global_NED.x
        state_msg.next_velocity_global_ned.y = self.next_velocity_global_NED.y
        state_msg.next_velocity_global_ned.z = self.next_velocity_global_NED.z

        state_msg.current_position_global_ned.x = self.current_position_global_NED.x
        state_msg.current_position_global_ned.y = self.current_position_global_NED.y
        state_msg.current_position_global_ned.z = self.current_position_global_NED.z

        state_msg.current_velocity_global_ned.x = self.current_velocity_global_NED.x
        state_msg.current_velocity_global_ned.y = self.current_velocity_global_NED.y
        state_msg.current_velocity_global_ned.z = self.current_velocity_global_NED.z

        # Ориентация
        
        state_msg.next_yaw_enu = self.next_yaw_ENU    
        state_msg.next_yaw_enu = self.next_yaw_ENU
        state_msg.current_yaw_enu = self.current_yaw_ENU        
        state_msg.next_yaw_ned = self.next_yaw_NED
        
        state_msg.current_yaw_ned = self.current_yaw_NED

        # Прочие состояния
        state_msg.offboard_mode = self.offboardMode
        state_msg.flight_check = self.flightCheck
        state_msg.arm_message = self.arm_message
        state_msg.failsafe = self.failsafe

        # Допустимые погрешности
        state_msg.position_tolerance = self.position_tolerance
        state_msg.orientation_tolerance = self.orientation_tolerance

        # Состояния
        state_msg.current_state = self.current_state
        state_msg.last_state = self.last_state

        # Публикуем сообщение состояния
        self.publisher_state_of_robot.publish(state_msg)

    def set_state(self, state_name: str):
        if state_name not in self.states:
            raise ValueError(f"Неизвестное состояние: {state_name}")
        self.current_state.exit()
        self.current_state = self.states[state_name]
        self.current_state.enter()

    def update_state(self):
        self.current_state.handle()

    def takeoff(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5)

    def loiter(self):
        pass

    def enable_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
                
    def disable_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 0.)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
    
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0)
        
    def take_off(self):        
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5.0)
    
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        
    def set_home_to_current_position(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_HOME, param1=1.0)


def main(args=None):
    rclpy.init(args=args)    
    control_node = X500BaseController()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        
        
        
        