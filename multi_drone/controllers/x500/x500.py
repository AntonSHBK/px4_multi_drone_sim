#!/usr/bin/env python3

import rclpy

from multi_drone.controllers.x500.x500_base import X500BaseController
from multi_drone.move_commands.x500.commander import X500Commander


class X500Controller(X500BaseController):
        
    def __init__(
        self,
        drone_id: int,
        drone_type: str,
        default_position=[0.0,0.0,0.0],
        default_orientation=[0.0,0.0,0.0]):                                    
        super().__init__(drone_id, drone_type, default_position, default_orientation) 
        self.gcode_commander = X500Commander(self, timer_execution=0.1)

def main(args=None):
    rclpy.init(args=args)    
    control_node = X500Controller(1, 'x500')
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        
        
        
        