#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import copy
import math
from collections import deque


class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')

        # === Parámetros ROS ===
        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_fixed')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('override_covariances', True)
        self.declare_parameter('gyro_scale_factor', 1.0)   # factor de corrección manual
        self.declare_parameter('apply_deg_to_rad', False)  # True si viene en grados/s
        self.declare_parameter('filter_window_size', 5)    # media móvil
        self.declare_parameter('calib_samples', 200)       # muestras para calibrar bias

        # === Cargar parámetros ===
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.override_covariances = bool(self.get_parameter('override_covariances').value)
        self.gyro_scale_factor = float(self.get_parameter('gyro_scale_factor').value)
        self.apply_deg_to_rad = bool(self.get_parameter('apply_deg_to_rad').value)
        self.filter_window_size = int(self.get_parameter('filter_window_size').value)
        self.calib_samples = int(self.get_parameter('calib_samples').value)

        # === Variables de calibración ===
        self.samples_seen = 0
        self.bias_z = 0.0
        self.calibrated = False

        # === Buffer para filtro ===
        self.angular_z_window = deque(maxlen=self.filter_window_size)

        # === Subs y pubs ===
        self.sub = self.create_subscription(Imu, self.input_topic, self.imu_callback, 10)
        self.pub = self.create_publisher(Imu, self.output_topic, 10)

        self.get_logger().info(
            f"✅ IMU Serial Node escuchando en {self.input_topic} → {self.output_topic} "
            f"(scale={self.gyro_scale_factor}, window={self.filter_window_size}, calib={self.calib_samples})"
        )

    # ------------------------------------------------------------------
    def imu_callback(self, msg: Imu):
        imu_msg = copy.deepcopy(msg)

        # === Ajustar timestamp y frame ===
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        # === Escalado manual (divide) ===
        if self.gyro_scale_factor != 1.0:
            imu_msg.angular_velocity.x /= self.gyro_scale_factor
            imu_msg.angular_velocity.y /= self.gyro_scale_factor
            imu_msg.angular_velocity.z /= self.gyro_scale_factor

        # === Convertir de °/s a rad/s si se requiere ===
        if self.apply_deg_to_rad:
            imu_msg.angular_velocity.x *= math.pi / 180.0
            imu_msg.angular_velocity.y *= math.pi / 180.0
            imu_msg.angular_velocity.z *= math.pi / 180.0

        # === Calibrar bias del giroscopio (solo las primeras N muestras) ===
        if not self.calibrated:
            if self.samples_seen < self.calib_samples:
                self.bias_z += imu_msg.angular_velocity.z
                self.samples_seen += 1
                return  # aún calibrando, no publicar
            else:
                self.bias_z /= float(self.calib_samples)
                self.calibrated = True
                self.get_logger().info(f"🧭 Calibrado bias wz = {self.bias_z:.6f} rad/s")

        # === Aplicar corrección de bias ===
        imu_msg.angular_velocity.z -= self.bias_z

        # === Filtro de media móvil sobre wz ===
        self.angular_z_window.append(imu_msg.angular_velocity.z)
        if len(self.angular_z_window) > 0:
            imu_msg.angular_velocity.z = sum(self.angular_z_window) / len(self.angular_z_window)

        # === Covarianzas ===
        if self.override_covariances:
            imu_msg.orientation_covariance[0] = -1.0
            imu_msg.angular_velocity_covariance = [
                0.5, 0.0, 0.0,
                0.0, 0.5, 0.0,
                0.0, 0.0, 0.5
            ]
            imu_msg.linear_acceleration_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]

        # === Publicar resultado ===
        self.pub.publish(imu_msg)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
