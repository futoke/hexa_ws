import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time

class HexaTeleop(Node):
    def __init__(self):
        super().__init__('hexa_teleop')
        
        # Подписка на джойстик
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Паблишеры
        self.cmd_vel_pub = self.create_publisher(Twist, '/joy/cmd_vel', 10)
        self.buttons_pub = self.create_publisher(Int32, '/joy/buttons', 10)
        
        # Состояние кнопок и время последнего нажатия для устранения дребезга
        self.last_button_times = {}  # Словарь для хранения времени последнего нажатия
        self.debounce_time = 0.2  # Время задержки в секундах (200 мс)

         # Флаг активности джойстика
        self.last_joy_msg = None
    
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

        # Публикация команды движения только если сообщение изменилось
        if self.last_joy_msg is None or self.last_joy_msg != msg:
            self.cmd_vel_pub.publish(twist)
            self.last_joy_msg = msg

        current_time = time.time()
        
        # Публикация нажатой кнопки (устранение дребезга по времени)
        for i, button in enumerate(msg.buttons):
            if button == 1:
                last_time = self.last_button_times.get(i, 0)
                if current_time - last_time > self.debounce_time:
                    button_msg = Int32()
                    button_msg.data = i  # Отправляем индекс нажатой кнопки
                    self.buttons_pub.publish(button_msg)
                    # self.get_logger().info(f'Button {i} pressed!')
                    self.last_button_times[i] = current_time  # Обновляем время последнего нажатия


def main(args=None):
    rclpy.init(args=args)
    node = HexaTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
