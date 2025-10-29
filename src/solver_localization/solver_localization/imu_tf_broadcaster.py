#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster


class ImuTFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_imu')

        # Suscriptor al topic de IMU
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.handle_imu_pose,
            10
        )

        # Broadcaster TF
        self.br = TransformBroadcaster(self)
        self.get_logger().info('📡 Nodo TF de IMU iniciado (odom → imu_link)')

    # ------------------------------------------------------------------
    def handle_imu_pose(self, msg: Imu):
        """Recibe orientación de la IMU y publica un transform TF."""
        try:
            orientation_list = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ]
            roll, pitch, yaw = euler_from_quaternion(orientation_list)
        except Exception:
            roll = pitch = yaw = 0.0

        # Crear transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'          # Frame padre (coincide con el EKF)
        t.child_frame_id = 'imu_link'       # Frame hijo

        # Sin traslación (IMU fija en el origen del robot)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Usar misma orientación recibida (ya es quaternion)
        quat = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Publicar TF
        self.br.sendTransform(t)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ImuTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Nodo TF detenido por el usuario.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
