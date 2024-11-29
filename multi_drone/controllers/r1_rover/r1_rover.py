import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import numpy as np

class RoverControl(Node):

    def __init__(self):
        super().__init__('rover_control')

        # Создаем подписку на команды движения (Twist), которые будут управлять роботом
        self.velocity_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )

        # Создаем подписку на данные лазерного сканера для предотвращения столкновений
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Создаем издателя для отправки команд на двигатели робота
        self.motor_pub = self.create_publisher(Twist, '/r1_rover/cmd_vel', 10)

        # Переменные для хранения текущих значений скорости и углового поворота
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Переменная для хранения состояния препятствий
        self.obstacle_detected = False

    def velocity_callback(self, msg):
        """Получаем команды скорости и углового поворота"""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.get_logger().info(f"Получена команда: линейная скорость {self.linear_velocity}, угловая скорость {self.angular_velocity}")

    def laser_callback(self, msg):
        """Обрабатываем данные лазерного сканера для обнаружения препятствий"""
        # Пример простейшего обнаружения препятствий (если расстояние меньше 0.5 метров)
        min_distance = np.min(msg.ranges)
        if min_distance < 0.5:
            self.obstacle_detected = True
            self.get_logger().info(f"Обнаружено препятствие на расстоянии {min_distance} метров")
        else:
            self.obstacle_detected = False

    def control_loop(self):
        """Цикл управления для отправки команд на робота"""
        # Если обнаружено препятствие, останавливаем робота
        if self.obstacle_detected:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.get_logger().info("Остановка из-за препятствия")

        # Публикуем команду движения
        cmd = Twist()
        cmd.linear.x = self.linear_velocity
        cmd.angular.z = self.angular_velocity
        self.motor_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RoverControl()

    # Запуск цикла управления с частотой 10 Гц
    timer_period = 0.1  # 100 мс
    node.create_timer(timer_period, node.control_loop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
