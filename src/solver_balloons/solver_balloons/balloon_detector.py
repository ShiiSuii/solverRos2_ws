#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from rclpy.action import ActionClient
import math
import time


class BalloonDetector(Node):

    def __init__(self):
        super().__init__("balloon_detector")

        self.bridge = CvBridge()

        # Subscriber cámara
        self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        # Cliente Nav2
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Marker RViz
        self.marker_pub = self.create_publisher(Marker, "/balloon_marker", 10)

        self.get_logger().info("Balloon detector listo.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # --- Detección por color (HSV) ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Rango ROJO (ajustable)
        lower = np.array([0, 120, 70])
        upper = np.array([10, 255, 255])

        mask1 = cv2.inRange(hsv, lower, upper)

        # Más rojo en el otro extremo del círculo HSV
        lower2 = np.array([170, 120, 70])
        upper2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower2, upper2)

        mask = mask1 | mask2

        # Encontrar contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return

        # Selecciona el globo más grande
        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)

        if area < 300:  # filtrar ruido
            return

        # Obtener centro del globo
        (x, y), radius = cv2.minEnclosingCircle(contour)
        img_h, img_w, _ = frame.shape
        cx, cy = int(x), int(y)

        # Publicar marcador RViz
        self.publish_marker()

        # Calcular ángulo del globo respecto al centro
        error_x = cx - img_w / 2
        angle = -error_x * (60.0 / img_w)  # cámara FOV 60°

        self.get_logger().info(f"Globo detectado. Ángulo: {angle:.2f}°")

        # Enviar navegación hacia un punto virtual
        self.navigate_towards(angle)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def navigate_towards(self, angle_deg):
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("Nav2 no está listo todavía.")
            return

        angle_rad = math.radians(angle_deg)

        # Movimiento simple hacia adelante
        target_x = math.cos(angle_rad) * 1.0
        target_y = math.sin(angle_rad) * 1.0

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "base_link"
        goal.pose.pose.position.x = target_x
        goal.pose.pose.position.y = target_y
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Navegando hacia x={target_x:.2f} y={target_y:.2f}")

        self.nav_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = BalloonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
