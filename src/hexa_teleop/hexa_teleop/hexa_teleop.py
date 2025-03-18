import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class HexaTeleop(Node):
    def __init__(self):
        super().__init__('hexa_teleop')
        
        # Подписка на джойстик
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Паблишеры
        self.cmd_vel_pub = self.create_publisher(Twist, '/joy/cmd_vel', 10)
        self.buttons_pub = self.create_publisher(Int32, '/joy/buttons', 10)
        
    def joy_callback(self, msg):
        twist = Twist()
        
        # Заполнение линейных скоростей
        twist.linear.x = msg.axes[3]  # X движение вперед-назад
        twist.linear.y = msg.axes[2]  # Y боковое движение
        twist.linear.z = 0.0  # Z движение вверх-вниз (если нужно)

        # Заполнение угловых скоростей
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = msg.axes[0]  # Поворот вокруг оси Z

        # Публикация команды движения
        self.cmd_vel_pub.publish(twist)

        # Публикация нажатой кнопки (если есть)
        for i, button in enumerate(msg.buttons):
            if button == 1:
                button_msg = Int32()
                button_msg.data = i  # Отправляем индекс нажатой кнопки
                self.buttons_pub.publish(button_msg)
                # self.get_logger().info(f'Button {i} pressed!')


def main(args=None):
    rclpy.init(args=args)
    node = HexaTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
