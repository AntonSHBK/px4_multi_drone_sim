from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist, Vector3, Point
from std_msgs.msg import Bool

from px4_msgs.msg import (
    OffboardControlMode, 
    TrajectorySetpoint, 
    VehicleStatus, 
    VehicleAttitude, 
    VehicleCommand,
)


class BaseDroneController(Node):
    def __init__(self, drone_type):
        
        self.drone_type = drone_type        
        
        super().__init__('base_controller')
                
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('default_position', "0,2,0,0,0,0")

        self.drone_id = self.get_parameter('drone_id').value
        
        default_position_str: str = self.get_parameter('default_position').value
        default_values = [float(x) for x in default_position_str.split(',')]
        self.default_world_position_ENU = Vector3(
            x=default_values[0],
            y=default_values[1],
            z=default_values[2]
            )
        self.default_world_orientation_ENU = default_values[3:]
        
        self.delta_position = Vector3(x=0., y=0., z=0.)
        self.delta_orientation = Vector3(x=0., y=0., z=0.)
        
        self.prefix_px = f'px4_{self.drone_id}'
        self.prefix_name = f'id_{self.drone_id}_{self.drone_type}'
        # self.name = f'{self.prefix_name}_controller_node'       
                        
        # QoS-профиль (менее надёжный)
        self.qos_profile_unreliable = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Достаточно лучшего усилия (без гарантии доставки)
            durability=QoSDurabilityPolicy.VOLATILE,  # Только актуальные сообщения
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # QoS-профиль (более надёжный)
        self.qos_profile_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Гарантированная доставка
            durability=QoSDurabilityPolicy.VOLATILE,  # Только актуальные сообщения
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        