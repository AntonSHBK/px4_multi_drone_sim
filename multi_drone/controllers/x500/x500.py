#!/usr/bin/env python3

import json
from dataclasses import dataclass
from tracemalloc import stop

from sympy import im
from geometry_msgs.msg import Point, Vector3
import numpy as np

import rclpy
from rclpy.clock import Clock
from std_msgs.msg import Bool, String

from px4_msgs.msg import (
    OffboardControlMode, 
    TrajectorySetpoint, 
    VehicleStatus, 
    VehicleAttitude, 
    VehicleCommand,
    VehicleLocalPosition
)

from multirobots.scripts.controllers.base_controller import BaseDroneController

from multirobots.scripts.move_commands import get_g_command_class
from multirobots.scripts.move_commands.base_command import BaseGCommand

from control_system.scripts.utils.methods import enu_to_ned
from control_system.scripts.utils.methods import ned_to_enu
from control_system.scripts.utils.methods import global_to_local
from control_system.scripts.utils.methods import local_to_global
from control_system.scripts.utils.methods import calculate_yaw_towards_target

from robots_service.msg import RobotState


class X500Controller(BaseDroneController):
        
    def __init__(self):
                    
        super().__init__('x500')
        
        self.get_logger().info(f"Инициализация дрона с ID {self.drone_id}, тип {self.drone_type}, позиция {self.default_world_position_ENU}")

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        
        # NED - СК дрона
        # ENU - СК мира
        
        # Локальные значения        
        self.next_position_local_ENU = Vector3(x=0.0, y=0.0, z=0.0)
        self.next_velocity_local_ENU = Vector3(x=0.0, y=0.0, z=0.0)
        
        self.next_position_local_NED = Vector3(x=0.0, y=0.0, z=-2.5)
        self.next_velocity_local_NED = Vector3(x=0.0, y=0.0, z=0.0)
        
        self.current_position_local_ENU = Vector3(x=0.0, y=0.0, z=0.0)
        self.current_velocity_local_ENU = Vector3(x=0.0, y=0.0, z=0.0)
        
        self.current_position_local_NED = Vector3(x=0.0, y=0.0, z=0.0)
        self.current_velocity_local_NED = Vector3(x=0.0, y=0.0, z=0.0)
        
        # Глобальные значения        
        self.next_position_global_ENU = Vector3(x=0.0, y=0.0, z=0.0)
        self.next_velocity_global_ENU = Vector3(x=0.0, y=0.0, z=0.0)
        
        self.next_position_global_NED = Vector3(x=0.0, y=0.0, z=0.0)
        self.next_velocity_global_NED = Vector3(x=0.0, y=0.0, z=0.0)
        
        self.current_position_global_ENU = Vector3(x=0.0, y=0.0, z=0.0)
        self.current_velocity_global_ENU = Vector3(x=0.0, y=0.0, z=0.0)
        
        self.current_position_global_NED = Vector3(x=0.0, y=0.0, z=0.0)
        self.current_velocity_global_NED = Vector3(x=0.0, y=0.0, z=0.0)
                
        self.next_yaw_ENU = 0.0       
        self.current_yaw_ENU = 0.0
        
        self.next_yaw_NED = 0.0        
        self.current_yaw_NED = 0.0
        
        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False
        
        self.position_tolerance = 0.1  # Допустимая погрешность для позиции (в метрах)
        self.orientation_tolerance = 0.05  # Допустимая погрешность для ориентации (в радианах)
        
        self.current_state = "IDLE"
        self.last_state = self.current_state
        
        timer_arm_period = 0.5
        self.timer_arm = self.create_timer(
            timer_arm_period, self.timer_arm_callback)

        timer_cmdloop_period = 0.005
        self.timer_cmdloop = self.create_timer(
            timer_cmdloop_period, self.timer_cmdloop_callback)
        
        timer_trajectory_command_period = 0.1
        self.timer_trajectory_command = self.create_timer(
            timer_trajectory_command_period, self.timer_trajectory_command_callback)
        
        timer_robot_of_state_period = 0.1
        self.timer_robot_of_state = self.create_timer(
            timer_robot_of_state_period, self.timer_robot_of_state_callback)
        
        self.control_mode = "trajectory"
        
        self.commands_data_list = []
        self.current_command_idx = 0
        self.last_sent_trajectory = None
        
        self.subscriber_arm_message = self.create_subscription(
            Bool,
            f'{self.prefix_name}/arm_message',
            self.arm_message_callback,
            self.qos_profile_reliable
        )
        
        self.subscriber_control_mode = self.create_subscription(
            String,
            f'{self.prefix_name}/control_mode',
            self.control_mode_callback,
            self.qos_profile_reliable
        )
        
        self.subscriber_offboard_cmd = self.create_subscription(
            TrajectorySetpoint,
            f'{self.prefix_name}/offboard_cmd',
            self.offboard_cmd_callback,
            self.qos_profile_reliable
        )
        
        self.trajectory_command_subscriber = self.create_subscription(
            String,
            f'{self.prefix_name}/trajectory_command_json',
            self.trajectory_command_callback,
            self.qos_profile_reliable
        )
        
        self.subscriber_vehicle_status = self.create_subscription(
            VehicleStatus,
            f'{self.prefix_px}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            self.qos_profile_unreliable
        )
        
        self.subscriber_vehicle_attitude = self.create_subscription(
            VehicleAttitude,
            f'{self.prefix_px}/fmu/out/vehicle_attitude',
            self.attitude_callback,
            self.qos_profile_unreliable
        )
        
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            f'{self.prefix_px}/fmu/out/vehicle_local_position',
            self.local_position_callback,
            self.qos_profile_unreliable
        )        
        
        self.publisher_state_of_robot = self.create_publisher(
            RobotState,
            f'{self.prefix_name}/out/state_of_robot',
            self.qos_profile_unreliable
        )
        
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            f'{self.prefix_px}/fmu/in/offboard_control_mode',
            self.qos_profile_reliable
        )
        
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            f'{self.prefix_px}/fmu/in/trajectory_setpoint',
            self.qos_profile_reliable
        )
        
        self.publisher_offboard_cmd = self.create_publisher(
            TrajectorySetpoint,
            f'{self.prefix_name}/offboard_cmd',
            self.qos_profile_reliable
        )
        
        self.publisher_vehicle_command = self.create_publisher(
            VehicleCommand,
            f'{self.prefix_px}/fmu/in/vehicle_command',
            self.qos_profile_reliable
        )
        
        self.set_home_to_current_position()

    def timer_arm_callback(self):
        # self.get_logger().info(f"Текущее состояние: {self.current_state}, состояние арминга: {self.arm_state}, навигационное состояние: {self.nav_state}")

        match self.current_state:
            case "IDLE":
                if self.flightCheck and self.arm_message:
                    self.current_state = "ARMING"
                    self.get_logger().info("Переход в состояние АРМИНГ")

            case "ARMING":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Проверка полета не пройдена, переход в состояние IDLE")
                elif self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10:
                    self.current_state = "TAKEOFF"
                    self.set_home_to_current_position()
                    self.get_logger().info("Армирование успешно, переход в состояние ВЗЛЕТ")
                self.arm()

            case "TAKEOFF":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Проверка полета не пройд`ена, переход в состояние IDLE")
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                    self.current_state = "LOITER"
                    self.get_logger().info("Взлет успешен, переход в состояние LOITER")
                self.arm()
                self.take_off()

            case "LOITER": 
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Проверка полета не пройдена, переход в состояние IDLE")
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    self.current_state = "OFFBOARD"
                    self.get_logger().info("Лоитер успешен, переход в состояние OFFBOARD")
                self.arm()

            case "OFFBOARD":
                if not self.flightCheck or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                    self.current_state = "IDLE"
                    self.get_logger().info("Offboard режим не выполнен, переход в состояние IDLE")
                self.state_offboard()

        if self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
            self.arm_message = False

        if self.last_state != self.current_state:
            self.last_state = self.current_state
            self.get_logger().info(f"Состояние изменено на: {self.current_state}")

        self.myCnt += 1
    
    def timer_cmdloop_callback(self):
        
        if self.offboardMode:
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            
            # TODO: разобраться какие ещё есть параметры для контроля (OffboardControlMode)

            if self.control_mode == "velocity":
                offboard_msg.position = False
                offboard_msg.velocity = True
                offboard_msg.acceleration = False
                self.publisher_offboard_mode.publish(offboard_msg)

                # Публикуем команду скорости
                self.publish_velocity_command()

            elif self.control_mode == "trajectory":
                offboard_msg.position = True
                offboard_msg.velocity = False
                offboard_msg.acceleration = False
                self.publisher_offboard_mode.publish(offboard_msg)

                # Публикуем команду трSаектории
                self.publish_position_setpoint()        
    
    def publish_velocity_command(self):
        pass    
        
    def publish_position_setpoint(self):
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        
        # Устанавливаем желаемые координаты
        trajectory_msg.position[0] = self.next_position_local_NED.x
        trajectory_msg.position[1] = self.next_position_local_NED.y
        trajectory_msg.position[2] = self.next_position_local_NED.z
        
        # Можно указать ускорение, если необходимо (либо оставить NaN)
        # trajectory_msg.acceleration[0] = float('nan')
        # trajectory_msg.acceleration[1] = float('nan')
        # trajectory_msg.acceleration[2] = float('nan')
        
        # Можно оставить скорость NaN, чтобы PX4 управлял этим автоматически
        # trajectory_msg.velocity[0] = self.next_velocity.x
        # trajectory_msg.velocity[1] = self.next_velocity.y
        # trajectory_msg.velocity[2] = self.next_velocity.z
        
        # Задаём желаемый угол (yaw)
        trajectory_msg.yaw = self.next_yaw_NED        
        trajectory_msg.yawspeed = float('nan')  # Оставляем автоматическое управление угловой скоростью
        
        # Публикуем сообщение
        self.publisher_trajectory.publish(trajectory_msg)
    
    def arm_message_callback(self, msg: Bool):
        self.arm_message = msg.data
        self.get_logger().info(f"Получено сообщение арминга: {self.arm_message}")
        
        # Если arm_message False, вызываем метод для посадки дрона
        if not self.arm_message:
            self.land()

    def control_mode_callback(self, msg):
        """Меняет режим управления на новый."""
        new_mode = msg.data
        if new_mode in ["velocity", "trajectory"]:
            self.control_mode = new_mode
            self.get_logger().info(f"Режим управления изменён на: {self.control_mode}")
        else:
            self.get_logger().warn(f"Неподдерживаемый режим управления: {new_mode}")
    
    # Callback-функция для получения и установки значений статуса дрона
    def vehicle_status_callback(self, msg: VehicleStatus):
        if msg.nav_state != self.nav_state:
            self.get_logger().info(f"NAV_STATUS изменён: {msg.nav_state}")
        
        if msg.arming_state != self.arm_state:
            self.get_logger().info(f"ARM_STATUS изменён: {msg.arming_state}")
            
        if msg.failsafe != self.failsafe:
            self.get_logger().info(f"Статус FAILSAFE изменён: {msg.failsafe}")
        
        if msg.pre_flight_checks_pass != self.flightCheck:
            self.get_logger().info(f"Статус проверки полета изменён: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    def offboard_cmd_callback(self, msg: TrajectorySetpoint):
        """Обработчик команд управления по траектории через TrajectorySetpoint с переводом в FLU."""

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
        
    def trajectory_command_callback(self, msg: String):
        try:
            data: list = json.loads(msg.data)
            for command_dict in data:
                command_name = command_dict.get('name')
                command_dict.pop("name", None)
                if command_name:
                    command: BaseGCommand = get_g_command_class(command_name, **command_dict)
                    # command.from_dict(command_dict)
                    self.stop_command_check(command=command)
                    self.commands_data_list.append(command)
                else:
                    self.get_logger().error("Отсутствует имя команды в данных")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Ошибка декодирования JSON: {e}")
    
    def stop_command_check(self, command: BaseGCommand):
        pass
    
    def timer_trajectory_command_callback(self):
        if self.commands_data_list:
            command: BaseGCommand  = self.get_next_command()
            if command:
                if not command.complete:
                    self.command_process(command)
                else:
                    self.command_complete()
    
    def  get_next_command(self) -> BaseGCommand:
        if self.current_command_idx < len(self.commands_data_list):
            return self.commands_data_list[self.current_command_idx]
        else:
            return None
    
    def command_complete(self):
        self.current_command_idx += 1
    
    def command_process(self, command: BaseGCommand):
        self.special_command(command)
        self.move_command(command)        
        
    def move_command(self, command: BaseGCommand):
        trajectory: TrajectorySetpoint = command.get_trajectory_setpoint()
        
        if self.check_position(trajectory):
            command.step_complete()
        # TODO Не работает условие, надо проверить
        elif not self.trajectories_are_equal(self.last_sent_trajectory, trajectory):
            self.publisher_offboard_cmd.publish(trajectory)
            self.last_sent_trajectory = trajectory
            # self.get_logger().info(f"Команда на перемещение отправлена: {trajectory}")        
    
    def trajectories_are_equal(self, trajectory1: TrajectorySetpoint, trajectory2: TrajectorySetpoint) -> bool:
        """
        Сравнивает значения двух объектов TrajectorySetpoint.
        Возвращает True, если значения position и yaw совпадают.
        """
        if not trajectory1 or not trajectory2:
            return False
        return (
            trajectory1.position[0] == trajectory2.position[0] and
            trajectory1.position[1] == trajectory2.position[1] and
            trajectory1.position[2] == trajectory2.position[2]
            # trajectory1.yaw == trajectory2.yaw
        )
    
    def special_command(self, command: BaseGCommand):
        pass
    
    def check_position(self, trajectory: TrajectorySetpoint) -> bool:
        # Проверка по позиции
        dx = abs(self.current_position_global_ENU.x - trajectory.position[0])
        dy = abs(self.current_position_global_ENU.y - trajectory.position[1])
        dz = abs(self.current_position_global_ENU.z - trajectory.position[2])
        
        position_within_tolerance = (
            dx <= self.position_tolerance and
            dy <= self.position_tolerance and
            dz <= self.position_tolerance
        )

        # Проверка по ориентации
        if not np.isnan(trajectory.yaw):
            d_yaw = abs(self.current_yaw_ENU - trajectory.yaw)
            orientation_within_tolerance = d_yaw <= self.orientation_tolerance
            return position_within_tolerance and orientation_within_tolerance                   
        
        return position_within_tolerance

    # Callback-функция для получения текущих значений траектории и извлечения угла рыскания
    def attitude_callback(self, msg: VehicleAttitude):
        """
        Обработчик для получения ориентации yaw дрона в системе ENU и преобразования в NED.
        """
        orientation_q = msg.q
        self.current_yaw_ENU = float(np.arctan2(2.0 * (orientation_q[3] * orientation_q[2] + orientation_q[0] * orientation_q[1]), 
                                  1.0 - 2.0 * (orientation_q[1] * orientation_q[1] + orientation_q[2] * orientation_q[2])))        
        self.current_yaw_NED = -self.current_yaw_ENU
    
    def local_position_callback(self, msg: VehicleLocalPosition):
        """
        Получение локальных координат дрона и высоты, обработка скоростей
        в системах координат NED и ENU.
        """
        
        self.current_position_local_NED.x = msg.x
        self.current_position_local_NED.y = msg.y
        self.current_position_local_NED.z = msg.z

        self.current_velocity_local_NED.x = msg.vx
        self.current_velocity_local_NED.y = msg.vy
        self.current_velocity_local_NED.z = msg.vz         

        position_local_ENU_x, position_local_ENU_y, position_local_ENU_z = ned_to_enu(
            self.current_position_local_NED.x, 
            self.current_position_local_NED.y, 
            self.current_position_local_NED.z
        )
        velocity_local_ENU_x, velocity_local_ENU_y, velocity_local_ENU_z = ned_to_enu(
            self.current_velocity_local_NED.x, 
            self.current_velocity_local_NED.y, 
            self.current_velocity_local_NED.z
        )
        
        self.current_position_local_ENU.x = position_local_ENU_x
        self.current_position_local_ENU.y = position_local_ENU_y
        self.current_position_local_ENU.z = position_local_ENU_z
        
        self.current_velocity_local_ENU.x = velocity_local_ENU_x
        self.current_velocity_local_ENU.y = velocity_local_ENU_y
        self.current_velocity_local_ENU.z = velocity_local_ENU_z
        
        self.current_position_global_ENU = local_to_global(
            self.current_position_local_ENU,
            self.default_world_position_ENU,
            self.default_world_orientation_ENU
        )
        
        self.current_velocity_global_ENU = local_to_global(
            self.current_velocity_local_ENU,
            Vector3(x=0.0, y=0.0, z=0.0),
            self.default_world_orientation_ENU
        )
        
        position_global_NED_x, position_global_NED_y, position_global_NED_z = enu_to_ned(
            self.current_position_global_ENU.x,
            self.current_position_global_ENU.y,
            self.current_position_global_ENU.z
        )
        velocity_global_NED_x, velocity_global_NED_y, velocity_global_NED_z = enu_to_ned(
            self.current_velocity_global_ENU.x,
            self.current_velocity_global_ENU.y,
            self.current_velocity_global_ENU.z
        )

        self.current_position_global_NED.x = position_global_NED_x
        self.current_position_global_NED.y = position_global_NED_y
        self.current_position_global_NED.z = position_global_NED_z

        self.current_velocity_global_NED.x = velocity_global_NED_x
        self.current_velocity_global_NED.y = velocity_global_NED_y
        self.current_velocity_global_NED.z = velocity_global_NED_z
        
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

    
    # Функции для инициализации состояния
    def state_init(self):
        self.myCnt = 0
        self.get_logger().info("Инициализация состояния")

    # Функция для состояния ARMED
    def state_arming(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Отправка команды армирования")

    # Функция для состояния TAKEOFF
    def state_takeoff(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5)  # param7 - высота в метрах
        self.get_logger().info("Отправка команды взлёта")

    # Функция для состояния LOITER
    def state_loiter(self):
        self.myCnt = 0
        self.get_logger().info("Статус лоитера: ожидание перехода")

    # Функция для состояния OFFBOARD
    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True        
        # self.get_logger().info("Включен Offboard режим")

    # Функция для отправки команды на армирование
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        # self.get_logger().info("Команда армирования отправлена")
    
    def set_home_to_current_position(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_HOME, param1=1.0)  # param1=1.0 — установить текущую позицию как "дом"
        # self.get_logger().info("Установлена текущая позиция как начало локальной системы координат")

    # def set_home_position(self, latitude, longitude, altitude):
    #     """
    #     Устанавливает конкретное местоположение как начальную точку (нулевую координату) локальной системы координат.
        
    #     :param latitude: Широта (в градусах)
    #     :param longitude: Долгота (в градусах)
    #     :param altitude: Высота (в метрах)
    #     """
    #     home_command = VehicleCommand()
    #     home_command.command = VehicleCommand.VEHICLE_CMD_DO_SET_HOME
    #     home_command.param1 = 0.0  # 0 = использовать переданные координаты, а не текущую позицию
    #     home_command.param2 = 0.0  # Зарезервировано, оставляем 0
    #     home_command.param3 = 0.0  # Зарезервировано, оставляем 0
    #     home_command.param4 = 0.0  # Зарезервировано, оставляем 0
    #     home_command.param5 = latitude  # Широта
    #     home_command.param6 = longitude  # Долгота
    #     home_command.param7 = altitude  # Высота

    #     home_command.target_system = self.drone_id + 1  # ID дрона
    #     home_command.target_component = 1
    #     home_command.source_system = self.drone_id
    #     home_command.source_component = 1
    #     home_command.from_external = True
    #     home_command.timestamp = int(Clock().now().nanoseconds / 1000)

    #     self.publisher_vehicle_command.publish(home_command)
    #     self.get_logger().info(f"Установлена начальная позиция: {latitude}, {longitude}, {altitude}")

    # Функция для отправки команды на взлет на заданную высоту
    def take_off(self):        
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5.0)  # param7 - высота в метрах
        self.get_logger().info("Команда взлёта отправлена")
    
    def land(self):
        """Отправляем команду на посадку дрона."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Команда на посадку отправлена")
        
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=5.0):
        msg = VehicleCommand()
        msg.param1 = param1  # Первичный параметр команды. Его значение зависит от типа команды (например, для команды армирования это может быть 1.0 для арма и 0.0 для дизарма)
        msg.param2 = param2  # Вторичный параметр команды. Используется в некоторых командах для дополнительной информации (например, для некоторых команд это может быть направление или время)
        msg.param7 = param7  # Значение высоты для команды на взлет (например, при взлёте указывает желаемую высоту в метрах)
        msg.command = command  # ID команды (например, команда армирования, взлёта, перехода в Offboard и т.д.)
        msg.target_system = self.drone_id  + 1 # Система, которая должна выполнить команду (обычно 1 для основного контроллера)
        msg.target_component = 1  # Компонент, который должен выполнить команду (обычно 1 для основного компонента)
        msg.source_system = self.drone_id  # Система, отправляющая команду (указываем 1, если команда отправляется с основного контроллера)
        msg.source_component = 1  # Компонент, отправляющий команду (обычно 1)
        msg.from_external = True  # Флаг, показывающий, что команда отправлена извне
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # Время отправки команды в микросекундах
        self.publisher_vehicle_command.publish(msg)  # Публикация команды)

def main(args=None):
    rclpy.init(args=args)    
    control_node = X500Controller()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        
        
        
        