#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class Rotate360(Node):
    def __init__(self):
        super().__init__('rotate_360')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # === Configuraciones ===
        self.angular_speed = 0.5   # rad/s (≈28.6°/s)
        self.duration = (2 * math.pi) / self.angular_speed  # tiempo para 360°

        self.get_logger().info(f'🔄 Iniciando giro 360° a {self.angular_speed:.2f} rad/s ({self.duration:.1f} s)')
        self.rotate()

    def rotate(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.angular_speed

        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz

        # Publicar durante la duración calculada
        while rclpy.ok() and (time.time() - start_time) < self.duration:
            self.pub.publish(msg)
            rate.sleep()

        # Detener
        msg.angular.z = 0.0
        self.pub.publish(msg)
        self.get_logger().info('✅ Giro completo (360°).')


def main(args=None):
    rclpy.init(args=args)
    node = Rotate360()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
