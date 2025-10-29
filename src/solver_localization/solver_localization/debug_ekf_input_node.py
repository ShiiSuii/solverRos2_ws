#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class EkfDebugNode(Node):
    def __init__(self):
        super().__init__('ekf_debug_node')

        self.sub_odom = self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data_fixed', self.imu_callback, 10)

        self.last_odom = None
        self.last_imu = None
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info('✅ EKF Debug Node iniciado: monitoreando /odom_raw y /imu/data_fixed')

    def odom_callback(self, msg):
        self.last_odom = msg

    def imu_callback(self, msg):
        self.last_imu = msg

    def timer_callback(self):
        if self.last_odom:
            vx = self.last_odom.twist.twist.linear.x
            wz = self.last_odom.twist.twist.angular.z
            self.get_logger().info(f'📘 ODOM: vx={vx:.3f} m/s, wz={wz:.3f} rad/s')
        if self.last_imu:
            wz_imu = self.last_imu.angular_velocity.z
            self.get_logger().info(f'🧭 IMU: wz={wz_imu:.3f} rad/s')
        if not self.last_odom and not self.last_imu:
            self.get_logger().warn('⚠️ Aún no se reciben datos de odometría ni IMU.')

def main(args=None):
    rclpy.init(args=args)
    node = EkfDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
