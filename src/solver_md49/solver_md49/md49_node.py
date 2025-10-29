#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
from solver_velocity.velocity_util import twist_to_wheels_rpm

class MD49Node(Node):
    """
    Nodo ROS2 para controlar el driver MD49 (roboclaw) y leer sus encoders.
    - Controla dos motores con /cmd_vel
    - Publica /lwheel y /rwheel (ticks de encoders)
    """

    def __init__(self):
        super().__init__('md49_node')

        # === Parámetros ===
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('wheel_radius_m', 0.075)
        self.declare_parameter('track_width_m', 1.65)
        self.declare_parameter('sample_rate', 10.0)  # Hz
    
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.wheel_radius = float(self.get_parameter('wheel_radius_m').value)
        self.track_width = float(self.get_parameter('track_width_m').value)
        self.rate = float(self.get_parameter('sample_rate').value)

        # 🔍 Mostrar confirmación de parámetros cargados
        self.get_logger().info(
            f"⚙️ Parámetros cargados → Rueda={self.wheel_radius:.3f} m | Base={self.track_width:.3f} m | Baud={baud} | Port={port}"
        )

        # === Conexión serial ===
        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"✅ MD49 conectado en {port} @ {baud} baudios")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ No se pudo abrir {port}: {e}")
            raise SystemExit

        # === Publicadores y suscriptores ===
        self.pub_lwheel = self.create_publisher(Int64, '/lwheel', 10)
        self.pub_rwheel = self.create_publisher(Int64, '/rwheel', 10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # === Timer de lectura de encoders ===
        self.timer = self.create_timer(1.0 / self.rate, self.read_encoders)

        # === Inicialización del MD49 ===
        self.stop_motors()
        self.reset_encoders()
        self.enable_regulator()
        self.enable_timeout()
        self.get_logger().info("🧭 Encoders reiniciados. Nodo listo.")

    # ----------------------------------------------------
    # Funciones de bajo nivel del MD49
    # ----------------------------------------------------
    def send_bytes(self, data):
        """Envía lista de bytes al MD49"""
        self.serial.write(bytearray(data))

    def stop_motors(self):
        """Detiene ambos motores (128 = neutro)"""
        self.send_bytes([0x00, 0x31, 128])
        self.send_bytes([0x00, 0x32, 128])
        self.get_logger().info("🛑 Motores detenidos (neutro 128).")

    def reset_encoders(self):
        """Resetea contadores de encoders"""
        self.send_bytes([0x00, 0x35])

    def enable_regulator(self):
        """Activa el regulador de velocidad"""
        self.send_bytes([0x00, 0x37])

    def enable_timeout(self):
        """Activa el corte de seguridad por inactividad"""
        self.send_bytes([0x00, 0x39])

    def set_motor_speeds(self, rpm_l, rpm_r):
        """Convierte RPM ±100 a rango 0–255 (128 neutro) y envía a los motores"""
        max_rpm = 100.0
        speed_l = int(128 + max(min(rpm_l / max_rpm, 1.0), -1.0) * 127)
        speed_r = int(128 + max(min(rpm_r / max_rpm, 1.0), -1.0) * 127)

        self.send_bytes([0x00, 0x31, speed_l])
        self.send_bytes([0x00, 0x32, speed_r])

    # ----------------------------------------------------
    # Callbacks
    # ----------------------------------------------------
    def cmd_callback(self, msg: Twist):
        """Convierte Twist (v, w) a velocidades por rueda"""
        v = msg.linear.x
        w = msg.angular.z

        rpm_l, rpm_r = twist_to_wheels_rpm(v, w, self.wheel_radius, self.track_width)
        self.get_logger().debug(
            f"cmd_vel: v={v:.2f} m/s, w={w:.2f} rad/s → L={rpm_l:.1f} RPM, R={rpm_r:.1f} RPM"
        )
        self.set_motor_speeds(rpm_l, rpm_r)

    def read_encoders(self):
        """Lee ambos encoders en una sola llamada (GET ENCODERS, 8 bytes)"""
        try:
            self.send_bytes([0x00, 0x25])
            data = self.serial.read(8)

            if len(data) != 8:
                self.get_logger().warn(f"⚠️ Bytes recibidos inválidos: {len(data)}")
                return

            enc_l = int.from_bytes(data[0:4], byteorder='big', signed=True)
            enc_r = int.from_bytes(data[4:8], byteorder='big', signed=True)

            self.pub_lwheel.publish(Int64(data=enc_l))
            self.pub_rwheel.publish(Int64(data=enc_r))

        except Exception as e:
            self.get_logger().warn(f"⚠️ Error lectura encoders: {e}")
            try:
                if not self.serial.is_open:
                    self.serial.open()
                    self.get_logger().info("🔄 Reconexion serial exitosa.")
            except Exception:
                pass

    def destroy_node(self):
        try:
            self.stop_motors()
            self.serial.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MD49Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Nodo detenido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
