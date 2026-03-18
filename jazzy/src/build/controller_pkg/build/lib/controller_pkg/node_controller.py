import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8, Float32
import serial
import struct
import time
import threading

class NodeController(Node):
    def __init__(self):
        super().__init__('node_controller')
        
        self.declare_parameter('port', '/dev/ttyArduino')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2.0) 
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"Connecté à l'Arduino sur {port}")
        except Exception as e:
            self.get_logger().error(f"Erreur connexion série : {e}")
            self.ser = None
        
        # Abonnement cmd_vel
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 1)
        
        # Publishers (Distances en mm)
        self.pub_ir_gauche = self.create_publisher(Float32, 'ir_gauche_mm', 10)
        self.pub_ir_droit = self.create_publisher(Float32, 'ir_droit_mm', 10)
        self.pub_servo_angle = self.create_publisher(UInt8, 'servo_angle', 10)
        
        if self.ser is not None:
            self.running = True
            self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
            self.read_thread.start()

    def raw_to_mm(self, raw_value):
        """
        Linéarisation pour capteur Sharp type GP2Y0A21 (10-80cm).
        La formule est : Distance = 1 / (coeff * ADC + offset)
        Approximation en mm :
        """
        if raw_value < 10: return 800.0 # Valeur trop faible / Hors portée
        
        try:
            # Formule de base pour conversion tension -> distance
            # (6787 / (v - 3)) - 4 en cm, ici adapté pour l'ADC 10 bits et mm
            distance_mm = (67870.0 / (raw_value - 3.0)) - 40.0
            
            # On contraint entre 100mm et 800mm (limites physiques du capteur)
            return max(100.0, min(800.0, distance_mm))
        except:
            return 800.0

    def listener_callback(self, msg):
        vitesse = int(max(min(msg.linear.x * 50 + 50, 100), 0))
        direction = int(max(min(msg.angular.z * 50 + 50, 100), 0))
        checksum = (vitesse + direction) & 0xFF 
        trame = struct.pack('BBBB', 0xFF, vitesse, direction, checksum)
        if self.ser: self.ser.write(trame)

    def read_serial_loop(self):
        buffer = bytearray()
        while self.running and rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer.extend(data)
                    
                    # Recherche du header et taille de 7 octets
                    while len(buffer) >= 7:
                        if buffer[0] == 0xDD:
                            self.parse_arduino_packet(buffer[:7])
                            buffer = buffer[7:]
                        else:
                            buffer.pop(0)
                else:
                    time.sleep(0.005)
            except Exception as e:
                self.get_logger().error(f'Erreur lecture série: {e}')

    def parse_arduino_packet(self, packet):
        # Reconstruction des ints 10 bits (MSB << 8 | LSB)
        raw_g = (packet[1] << 8) | packet[2]
        raw_d = (packet[3] << 8) | packet[4]
        angle_servo = packet[5]
        checksum_recu = packet[6]
        
        # Vérif Checksum (Somme des 5 octets de données)
        calc_sum = sum(packet[1:6]) & 0xFF
        
        if calc_sum != checksum_recu:
            return

        # Publication IR Gauche
        msg_g = Float32()
        msg_g.data = self.raw_to_mm(raw_g)
        self.pub_ir_gauche.publish(msg_g)
        
        # Publication IR Droit
        msg_d = Float32()
        msg_d.data = self.raw_to_mm(raw_d)
        self.pub_ir_droit.publish(msg_d)
        
        # Publication Servo
        msg_s = UInt8()
        msg_s.data = angle_servo
        self.pub_servo_angle.publish(msg_s)

    def destroy_node(self):
        self.running = False
        if self.ser: self.ser.close()
        super().destroy_node()

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
