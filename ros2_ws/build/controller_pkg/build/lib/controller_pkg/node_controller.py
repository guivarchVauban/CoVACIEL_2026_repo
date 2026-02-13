import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import time

class NodeController(Node):
    def __init__(self):
        super().__init__('node_controller')
        
        # Configuration du port série pour l'Arduino
        self.declare_parameter('port', '/dev/ttyArduino')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(1.0) #test a enlever des que possible
            self.ser.reset_input_buffer() #test aussi
            self.get_logger().info(f"Arduino prêt a la connexion")
            self.get_logger().info(f"Connecté à l'Arduino sur {port}")
        except Exception as e:
            self.get_logger().error(f"Erreur connexion série sur {port} : {e}")

        # Abonnement au topic cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            1)

    def listener_callback(self, msg):
        # On récupère les axes souhaités
        lin_x = msg.linear.x
        ang_z = msg.angular.z

        # MAPPING : On transforme -1.0 / 1.0 en échelle 0 - 100
        # Formule : $$vitesse = (x \times 50) + 50$$
        vitesse = int(max(min(lin_x * 50 + 50, 100), 0))
        direction = int(max(min(ang_z * 50 + 50, 100), 0))

        # Calcul du Checksum (Octet 2 + Octet 3)
        checksum = (vitesse + direction) & 0xFF 

        # Construction de la trame binaire : [START, VITESSE, DIRECTION, CHECKSUM]
        # 'BBBB' signifie 4 octets non signés (unsigned char)
        trame = struct.pack('BBBB', 0xFF, vitesse, direction, checksum)

        self.ser.write(trame)

def main(args=None):
    rclpy.init(args=args)
    node = NodeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
