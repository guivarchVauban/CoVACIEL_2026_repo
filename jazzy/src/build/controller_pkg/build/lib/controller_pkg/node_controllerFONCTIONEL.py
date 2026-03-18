import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
import serial
import struct
import time
import threading


class NodeController(Node):
    def __init__(self):
        super().__init__('node_controller')
        
        # Configuration du port série pour l'Arduino
        # Note: Dans Docker, le port est mappé vers /dev/ttyArduino via docker-compose
        # En dehors de Docker, utiliser le paramètre: -p port:=/dev/serial/by-id/...
        self.declare_parameter('port', '/dev/ttyArduino')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2.0) 
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(0.5)
            self.get_logger().info(f"Connecté à l'Arduino sur {port}")
        except Exception as e:
            self.get_logger().error(f"Erreur connexion série sur {port} : {e}")
            self.ser = None
        
        # Abonnement au topic cmd_vel (EXISTANT - INCHANGÉ)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            1)
        
        # ========== NOUVEAU : Publishers pour les données Arduino ==========
        self.pub_ir_gauche = self.create_publisher(UInt8, 'ir_gauche', 10)
        self.pub_ir_droit = self.create_publisher(UInt8, 'ir_droit', 10)
        self.pub_servo_angle = self.create_publisher(UInt8, 'servo_angle', 10)
        
        # ========== NOUVEAU : Thread de lecture série ==========
        if self.ser is not None:
            self.running = True
            self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
            self.read_thread.start()
            self.get_logger().info("Thread de lecture série démarré")
    
    def listener_callback(self, msg):
        """EXISTANT - INCHANGÉ : Envoi des commandes vers Arduino"""
        # On récupère les axes souhaités
        lin_x = msg.linear.x
        ang_z = msg.angular.z
        
        # MAPPING : On transforme -1.0 / 1.0 en échelle 0 - 100
        vitesse = int(max(min(lin_x * 50 + 50, 100), 0))
        direction = int(max(min(ang_z * 50 + 50, 100), 0))
        
        # Calcul du Checksum (Octet 2 + Octet 3)
        checksum = (vitesse + direction) & 0xFF 
        
        # Construction de la trame binaire : [START, VITESSE, DIRECTION, CHECKSUM]
        trame = struct.pack('BBBB', 0xFF, vitesse, direction, checksum)
        
        # Logging
        self.get_logger().info(f"Envoi -> V:{vitesse} D:{direction} | Trame: {trame.hex(' ')}")
        
        try:
            if self.ser is not None:
                self.ser.write(trame)
        except Exception as e:
            self.get_logger().error(f"Erreur d'écriture série : {e}")
    
    # ========== NOUVEAU : Lecture des données Arduino ==========
    def read_serial_loop(self):
        """Thread de lecture continue du port série pour recevoir les données IR + Servo"""
        buffer = bytearray()
        
        while self.running and rclpy.ok():
            try:
                # Lecture des données disponibles
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer.extend(data)
                    
                    # Recherche du header 0xDD
                    while len(buffer) >= 5:  # Taille minimale d'un paquet
                        # Cherche le header
                        if buffer[0] == 0xDD:
                            # Paquet complet trouvé
                            if len(buffer) >= 5:
                                self.parse_arduino_packet(buffer[:5])
                                buffer = buffer[5:]  # Supprime le paquet traité
                            else:
                                break  # Attendre plus de données
                        else:
                            # Byte invalide, on avance d'un cran
                            buffer.pop(0)
                else:
                    # Petite pause si pas de données (évite 100% CPU)
                    time.sleep(0.001)
            
            except Exception as e:
                self.get_logger().error(f'Erreur lecture série: {e}')
                time.sleep(0.1)
    
    def parse_arduino_packet(self, packet):
        """
        Parse un paquet reçu de l'Arduino
        Format: [0xDD, IR_G, IR_D, Angle_Servo, Checksum]
        """
        if len(packet) != 5:
            return
        
        header = packet[0]
        ir_gauche = packet[1]
        ir_droit = packet[2]
        angle_servo = packet[3]
        checksum_recu = packet[4]
        
        # Vérification du checksum
        checksum_calc = (ir_gauche + ir_droit + angle_servo) & 0xFF
        
        if checksum_calc != checksum_recu:
            self.get_logger().warn(
                f'Checksum invalide: calculé={checksum_calc} reçu={checksum_recu}'
            )
            return
        
        # Publication des données sur les topics ROS2
        msg_ir_g = UInt8()
        msg_ir_g.data = ir_gauche
        self.pub_ir_gauche.publish(msg_ir_g)
        
        msg_ir_d = UInt8()
        msg_ir_d.data = ir_droit
        self.pub_ir_droit.publish(msg_ir_d)
        
        msg_servo = UInt8()
        msg_servo.data = angle_servo
        self.pub_servo_angle.publish(msg_servo)
        
        # Log optionnel (commenter en production si trop verbeux)
        # self.get_logger().info(
        #     f'RX <- IR_G={ir_gauche} IR_D={ir_droit} Servo={angle_servo}°'
        # )
    
    def destroy_node(self):
        """Nettoyage à la fermeture"""
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join(timeout=1.0)
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Port série fermé")
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
