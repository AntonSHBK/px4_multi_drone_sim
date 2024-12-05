from abc import ABC, abstractmethod

from px4_msgs.msg import (
    VehicleStatus, 
)

from multi_drone.controllers.x500.x500_base import X500BaseController, X500Params

class DroneState(ABC):
    """
    Базовый класс для всех состояний дрона.
    """
    def __init__(self, controller: X500BaseController):        
        self.controller = controller
        self.params = controller.params

    @abstractmethod
    def enter(self):
        """
        Метод вызывается при входе в состояние.
        """
        pass

    @abstractmethod
    def handle(self):
        """
        Метод вызывается для обработки логики текущего состояния.
        """
        pass

    @abstractmethod
    def exit(self):
        """
        Метод вызывается при выходе из состояния.
        """
        pass


class IdleState(DroneState):
    """
    Состояние ожидания (IDLE).
    """
    def enter(self):
        self.controller.log_info("Переход в состояние IDLE.")

    def handle(self):
        if self.params.flightCheck and self.params.arm_message:
            self.controller.set_state("ARMING")

    def exit(self):
        self.controller.log_info("Выход из состояния IDLE.")


class ArmingState(DroneState):
    """
    Состояние армирования дрона.
    """
    def enter(self):
        self.controller.log_info("Переход в состояние ARMING.")
        self.controller.arm()

    def handle(self):
        if not self.params.flightCheck:
            self.controller.set_state("IDLE")
        elif self.params.arm_state ==  VehicleStatus.ARMING_STATE_ARMED:
            self.controller.set_state("TAKEOFF")

    def exit(self):
        self.controller.log_info("Выход из состояния ARMING.")


class TakeoffState(DroneState):
    """
    Состояние взлета.
    """
    def enter(self):
        self.controller.log_info("Переход в состояние TAKEOFF.")
        self.controller.takeoff() 

    def handle(self):
        if not self.params.flightCheck:
            self.controller.set_state("IDLE")
        elif self.params.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
            self.controller.set_state("LOITER")

    def exit(self):
        self.controller.log_info("Выход из состояния TAKEOFF.")


class LoiterState(DroneState):
    """
    Состояние удержания позиции (LOITER).
    """
    def enter(self):
        self.controller.log_info("Переход в состояние LOITER.")
        # Возможно, публикация команды на удержание позиции

    def handle(self):
        if not self.params.flightCheck:
            self.controller.set_state("IDLE")
        elif self.params.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            if self.params.offboard_mode:
                self.controller.set_state("OFFBOARD")
            elif self.params.landing:
                self.controller.set_state("LANDING")

    def exit(self):
        self.controller.log_info("Выход из состояния LOITER.")

class OffboardState(DroneState):
    """
    Состояние управления в режиме Offboard.
    """
    def enter(self):
        self.controller.log_info("Переход в состояние OFFBOARD.")
        self.controller.enable_offboard_mode()

    def handle(self):
        if not self.params.flightCheck or self.params.failsafe:
            self.controller.set_state("IDLE")
        
        elif not self.params.offboard_mode:
            self.controller.set_state("LOITER")

    def exit(self):
        self.controller.log_info("Выход из состояния OFFBOARD.")
        self.controller.disable_offboard_mode()


class LandingState(DroneState):
    """
    Состояние посадки.
    """
    def enter(self):
        self.controller.log_info("Переход в состояние LANDING.")
        self.controller.land()

    def handle(self):
        if self.params.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            self.controller.set_state("DISARM")

    def exit(self):
        self.controller.log_info("Выход из состояния LANDING.")


class DisarmState(DroneState):
    """
    Состояние разармирования.
    """
    def enter(self):
        self.controller.log_info("Переход в состояние DISARM.")
        self.controller.disarm()

    def handle(self):
        if self.controller.params.arm_state == VehicleStatus.ARMING_STATE_STANDBY:
            self.controller.set_state("IDLE")

    def exit(self):
        self.controller.log_info("Выход из состояния DISARM.")
