import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class HexaTeleop(Node):
    def __init__(self):
        super().__init__('hexa_teleop')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/joy/cmd_vel', 10)
        
        self.z_offset = 0.07
        self.x_offset = 0.18
        self.max_step_len = 0.0
        self.step_speed = 1.875
    
    def joy_callback(self, msg):
        twist = Twist()
        
        # Обработка линейных скоростей
        twist.linear.x = msg.axes[3] * self.max_step_len * 2  # X
        twist.linear.y = msg.axes[2] * self.max_step_len * 2  # Y
        twist.linear.z = 0.0  # Z

        # Обработка угловых скоростей
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = msg.axes[0] * 0.05  # Поворот вокруг Z

        self.step_speed = 0.75 + (msg.axes[4] + 1) * (3.0 - 0.75) / 2.0

        # Обработка кнопок
        if msg.buttons[4] == 1:
            if self.z_offset > 0.025:
                self.z_offset -= 0.005
        if msg.buttons[0] == 1:
            if self.z_offset < 0.15:
                self.z_offset += 0.005
        if msg.buttons[1] == 1:
            if self.x_offset < 0.20:
                self.x_offset += 0.005
        if msg.buttons[3] == 1:
            if self.x_offset > 0.01:
                self.x_offset -= 0.005

        # Публикация команды
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = HexaTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
