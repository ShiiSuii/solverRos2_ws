#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav2_msgs.srv import SaveMap

class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_save_on_button')

        # Suscripción al joystick
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Cliente al servicio de guardado
        self.client = self.create_client(SaveMap, '/map_saver/save_map')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Esperando al servicio /map_saver/save_map...")

        self.last_button_state = 0  # Evitar guardar varias veces por mantenerlo apretado

        self.get_logger().info("Nodo listo: presiona TRIÁNGULO para guardar el mapa.")

    def joy_callback(self, msg: Joy):
        triangle = msg.buttons[3]  # TRIÁNGULO PS4

        # Detectar flanco ascendente (cuando pasa de 0 → 1)
        if triangle == 1 and self.last_button_state == 0:
            self.save_map()

        self.last_button_state = triangle

    def save_map(self):
        request = SaveMap.Request()
        request.map_url = "/home/athome/mapa_solver"
        request.map_topic = "map"
        request.occupied_thresh = 0.65
        request.free_thresh = 0.25

        self.get_logger().info("🟪 TRIÁNGULO presionado → Guardando mapa...")

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            result = future.result()
            if result.result:
                self.get_logger().info("✅ ¡Mapa guardado correctamente!")
            else:
                self.get_logger().error("❌ Falló el guardado del mapa.")
        except Exception as e:
            self.get_logger().error(f"Error al guardar el mapa: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
