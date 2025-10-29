#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist

class SolverVelocityNode(Node):
    def __init__(self):
        super().__init__('solver_velocity_node')

        # --- Parámetros ---
        self.declare_parameter('wheel_radius_m', 0.075)
        self.declare_parameter('track_width_m', 1.65)   
        self.declare_parameter('ticks_per_rev', 370.0)
        self.declare_parameter('sample_period', 0.1)

        self.r = self.get_parameter('wheel_radius_m').value
        self.b = self.get_parameter('track_width_m').value
        self.tpr = self.get_parameter('ticks_per_rev').value
        self.dt = self.get_parameter('sample_period').value

        # --- Estado interno ---
        self.last_l = None
        self.last_r = None
        self.prev_l = None
        self.prev_r = None

        # --- ROS ---
        self.create_subscription(Int64, '/lwheel', self.lwheel_cb, 10)
        self.create_subscription(Int64, '/rwheel', self.rwheel_cb, 10)
        self.vel_pub = self.create_publisher(Twist, '/vel_raw', 10)

        self.timer = self.create_timer(self.dt, self.update_velocity)

        self.get_logger().info('✅ solver_velocity_node inicializado correctamente')

    def lwheel_cb(self, msg):
        self.last_l = msg.data

    def rwheel_cb(self, msg):
        self.last_r = msg.data

    def update_velocity(self):
        if self.last_l is None or self.last_r is None:
            return

        if self.prev_l is None or self.prev_r is None:
            self.prev_l = self.last_l
            self.prev_r = self.last_r
            return

        dl_ticks = self.last_l - self.prev_l
        dr_ticks = self.last_r - self.prev_r

        self.prev_l = self.last_l
        self.prev_r = self.last_r

        # Convertir ticks a distancia
        dist_l = (dl_ticks / self.tpr) * (2 * math.pi * self.r)
        dist_r = (dr_ticks / self.tpr) * (2 * math.pi * self.r)

        # Velocidades lineales de cada rueda
        v_l = dist_l / self.dt
        v_r = dist_r / self.dt

        # Cinemática diferencial
        v = (v_r + v_l) / 2.0
        w = (v_r - v_l) / self.b

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w

        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SolverVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Nodo detenido por el usuario.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
