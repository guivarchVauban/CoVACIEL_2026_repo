import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')
        try:
            self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
            self.get_logger().info("Connecte a l'Arduino sur /dev/ttyACM0 !")
        except Exception as e:
            self.get_logger().error(f"Erreur USB : {e}")
        self.subscription = self.create_subscription(Int32, '/serv_cmd', self.callback, 10)

    def callback(self, msg):
        if hasattr(self, 'ser'):
            self.ser.write(f"STEER {msg.data}\n".encode())
            self.get_logger().info(f"Envoi Arduino : STEER  {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArduinoBridge())
    rclpy.shutdown()
