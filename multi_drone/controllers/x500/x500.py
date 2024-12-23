#!/usr/bin/env python3

import rclpy

from multi_drone.controllers.x500.x500_base import X500BaseController
from multi_drone.move_commands.x500.commander import X500Commander


class X500Controller(X500BaseController):
        
    def __init__(
        self,
        drone_id: int = 1,
        drone_type: str = 'x500',
        default_position: list = [0.0,0.0,0.0],
        default_orientation: list = [0.0,0.0,0.0]):                                    
        super().__init__(
            drone_id=drone_id, 
            drone_type=drone_type, 
            default_position=default_position,
            default_orientation=default_orientation
        )        
        self.g_code_commander = X500Commander(self, timer_execution=0.1)
    
def main():    
    rclpy.init()     
    control_node = X500Controller()
    
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        control_node.get_logger().info("Остановка контроллера.")
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        
        
        
        