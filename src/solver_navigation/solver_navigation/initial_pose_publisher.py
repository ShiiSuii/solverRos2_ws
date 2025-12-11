#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # Publicador de pose inicial
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # Publica 2 veces para asegurar que AMCL lo reciba
        self.count = 0
        self.timer = self.create_timer(1.0, self.publish_pose)

    def publish_pose(self):
        if self.count >= 2:
            self.get_logger().info("Initial pose publishing completed.")
            rclpy.shutdown()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"

        # Pose inicial
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # Covarianza mínima para evitar warnings en AMCL
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25       # x
        msg.pose.covariance[7] = 0.25       # y
        msg.pose.covariance[35] = 0.068538  # yaw (≈15°)

        self.pub.publish(msg)
        self.get_logger().info(f"Initial pose published ({self.count+1}/2)")

        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)   # deja correr el timer normalmente
    node.destroy_node()


if __name__ == '__main__':
    main()
