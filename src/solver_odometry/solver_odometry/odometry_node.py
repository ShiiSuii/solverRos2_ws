#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3, Pose
from nav_msgs.msg import Odometry


class SolverOdometry(Node):
    def __init__(self):
        super().__init__('solver_odometry')

        # === Parámetros configurables ===
        self.declare_parameter('frame_id', 'odom_raw')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('cov_lin', 0.02)
        self.declare_parameter('cov_ang', 0.05)

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.cov_lin = float(self.get_parameter('cov_lin').value)
        self.cov_ang = float(self.get_parameter('cov_ang').value)

        # === Estado interno ===
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_theta = 0.0
        self.vel_lin = 0.0
        self.vel_ang = 0.0
        self.last_time = self.get_clock().now()

        # === Interfaces ROS ===
        self.create_subscription(Twist, '/vel_raw', self.vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.timer = self.create_timer(0.05, self.update_odom)

        self.get_logger().info('✅ solver_odometry inicializado correctamente')

    # === Callback de velocidades ===
    def vel_callback(self, msg: Twist):
        self.vel_lin = msg.linear.x
        self.vel_ang = msg.angular.z

    # === Integrador de posición ===
    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        # Protección ante saltos de tiempo o valores NaN
        if dt <= 0.0 or math.isnan(dt) or dt > 1.0:
            return
        self.last_time = now

        # Validación de velocidades
        if any(math.isnan(v) for v in [self.vel_lin, self.vel_ang]):
            self.get_logger().warn('⚠️ Velocidades con NaN, descartando ciclo')
            return

        # Integración diferencial
        delta_x = self.vel_lin * math.cos(self.pos_theta) * dt
        delta_y = self.vel_lin * math.sin(self.pos_theta) * dt
        delta_th = self.vel_ang * dt

        if any(math.isnan(v) for v in [delta_x, delta_y, delta_th]):
            self.get_logger().warn('⚠️ Delta con NaN, descartando ciclo')
            return

        self.pos_x += delta_x
        self.pos_y += delta_y
        self.pos_theta += delta_th

        # Normalizar ángulo a [-pi, pi]
        self.pos_theta = math.atan2(math.sin(self.pos_theta), math.cos(self.pos_theta))

        # Generar quaternion siempre válido
        qz = math.sin(self.pos_theta / 2.0)
        qw = math.cos(self.pos_theta / 2.0)
        if any(math.isnan(v) for v in [qz, qw]) or (qz == 0.0 and qw == 0.0):
            qz, qw = 0.0, 1.0

        # === Mensaje Odometry ===
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        # Pose
        odom.pose.pose = Pose(
            position=Point(x=self.pos_x, y=self.pos_y, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        )

        # Twist (velocidades)
        odom.twist.twist = Twist(
            linear=Vector3(x=self.vel_lin, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=self.vel_ang)
        )

        # === Covarianzas (36 elementos) ===
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = self.cov_lin
        odom.pose.covariance[7] = self.cov_lin
        odom.pose.covariance[35] = self.cov_ang

        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = self.cov_lin
        odom.twist.covariance[7] = self.cov_lin
        odom.twist.covariance[35] = self.cov_ang

        # Publicar solo si todo es válido
        if any(math.isnan(v) for v in [self.pos_x, self.pos_y, self.pos_theta]):
            self.get_logger().warn('⚠️ Pose inválida, descartando publicación')
            return

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SolverOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Nodo detenido por el usuario.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
