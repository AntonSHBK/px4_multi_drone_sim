import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from multi_drone.move_commands.x500.g_code.g_code import G20_MoveToPoint

class TestMoveToPointNode(Node):
    """
    Нода для отправки тестовой команды движения к точке в глобальной системе координат ENU.
    """
    def __init__(self):
        super().__init__('test_move_to_point_node')

        self.target_point = G20_MoveToPoint(
            x=0,
            y=0,
            z=10,
            coordinate_system='global_ENU'
        )
        
        self.command_publisher = self.create_publisher(
            String,
            '/px4_1/command_json',
            10 
        )
        try:
            command_msg = String()
            command_msg.data = json.dumps(self.target_point.to_dict())
            self.command_publisher.publish(command_msg)
            self.get_logger().info(f"Отправлена команда: {command_msg.data}")
        except Exception as e:
            self.get_logger().error(f"Ошибка при отправке команды: {e}")



def main(args=None):
    rclpy.init(args=args)

    try:
        node = TestMoveToPointNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
