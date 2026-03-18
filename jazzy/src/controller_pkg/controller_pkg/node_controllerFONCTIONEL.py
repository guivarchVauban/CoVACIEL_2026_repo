import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8, UInt16  # MODIFIÉ : ajout UInt16
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
        
        # Publishers pour les données Arduino
        # MODIFIÉ : UInt16 pour IR (valeurs en mm, potentiellement > 255)
        self.pub_ir_gauche  = self.create_publisher(UInt16, 'ir_gauche', 10)
        self.pub_ir_droit   = self.create_publisher(UInt16, 'ir_droit', 10)
        self.pub_servo_angle = self.create_publisher(UInt8, 'servo_angle', 10)  # Inchangé
        
        # Thread de lecture série
        if self.ser is not None:
            self.running = True
            self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
            self.read_thread.start()
            self.get_logger().info("Thread de lecture série démarré")
    
    def listener_callback(self, msg):
        """EXISTANT - INCHANGÉ : Envoi des commandes vers Arduino
        Format TX: [0xFF, VITESSE, DIRECTION, CHECKSUM] = 4 octets
        """
        lin_x = msg.linear.x
        ang_z = msg.angular.z
        
        # MAPPING : On transforme -1.0 / 1.0 en échelle 0 - 100
        vitesse   = int(max(min(lin_x * 50 + 50, 100), 0))
        direction = int(max(min(ang_z * 50 + 50, 100), 0))
        
        checksum = (vitesse + direction) & 0xFF 
        trame = struct.pack('BBBB', 0xFF, vitesse, direction, checksum)
        
        self.get_logger().info(f"Envoi -> V:{vitesse} D:{direction} | Trame: {trame.hex(' ')}")
        
        try:
            if self.ser is not None:
                self.ser.write(trame)
        except Exception as e:
            self.get_logger().error(f"Erreur d'écriture série : {e}")
    
    def read_serial_loop(self):
        """Thread de lecture continue du port série.
        Format RX attendu: [0xDD, IG_H, IG_L, ID_H, ID_L, Angle, CS] = 7 octets
        """
        buffer = bytearray()
        
        while self.running and rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer.extend(data)
                    
                    # MODIFIÉ : taille minimale = 7 octets (était 5)
                    while len(buffer) >= 7:
                        if buffer[0] == 0xDD:
                            self.parse_arduino_packet(buffer[:7])  # MODIFIÉ : [:7]
                            buffer = buffer[7:]                    # MODIFIÉ : [7:]
                        else:
                            buffer.pop(0)  # Octet parasite, on avance
                else:
                    time.sleep(0.001)
            
            except Exception as e:
                self.get_logger().error(f'Erreur lecture série: {e}')
                time.sleep(0.1)
    
    def parse_arduino_packet(self, packet):
        """Parse un paquet reçu de l'Arduino.

        Format: [0xDD, IG_H, IG_L, ID_H, ID_L, Angle, CS]
          - IR Gauche  = (packet[1] << 8) | packet[2]  → uint16, en mm
          - IR Droit   = (packet[3] << 8) | packet[4]  → uint16, en mm
          - Angle servo = packet[5]                    → uint8, en degrés
          - Checksum   = packet[6] == (IG_H+IG_L+ID_H+ID_L+Angle) & 0xFF
        """
        if len(packet) != 7:
            return
        
        # Reconstruction des uint16 depuis MSB/LSB
        ir_gauche   = (packet[1] << 8) | packet[2]
        ir_droit    = (packet[3] << 8) | packet[4]
        angle_servo = packet[5]
        checksum_recu = packet[6]
        
        # Vérification checksum
        checksum_calc = (packet[1] + packet[2] + packet[3] + packet[4] + angle_servo) & 0xFF
        
        if checksum_calc != checksum_recu:
            self.get_logger().warn(
                f'Checksum invalide: calculé={checksum_calc} reçu={checksum_recu}'
            )
            return
        
        # Publication IR en UInt16 (mm)
        msg_ir_g = UInt16()
        msg_ir_g.data = ir_gauche
        self.pub_ir_gauche.publish(msg_ir_g)
        
        msg_ir_d = UInt16()
        msg_ir_d.data = ir_droit
        self.pub_ir_droit.publish(msg_ir_d)
        
        # Publication servo en UInt8 (degrés) — inchangé
        msg_servo = UInt8()
        msg_servo.data = angle_servo
        self.pub_servo_angle.publish(msg_servo)
        
        # Log optionnel (commenter en production si trop verbeux)
        # self.get_logger().info(
        #     f'RX <- IR_G={ir_gauche}mm IR_D={ir_droit}mm Servo={angle_servo}°'
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
