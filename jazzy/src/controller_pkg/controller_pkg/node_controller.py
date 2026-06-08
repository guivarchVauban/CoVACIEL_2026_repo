#!/usr/bin/env python3

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
        
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        
        self.ser = None
        self.ser_lock = threading.Lock()
        self.running = True
        
        # Publishers
        self.pub_ir_gauche   = self.create_publisher(Float32, 'ir_gauche', 10)
        self.pub_ir_droit    = self.create_publisher(Float32, 'ir_droit', 10)
        self.pub_servo_angle = self.create_publisher(UInt8,  'servo_angle', 10)
        
        # Abonnement cmd_vel
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 1)
        
        # Tentative de connexion initiale
        self._try_connect()
        
        # Thread de lecture (tourne même si non connecté, gère la reconnexion)
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()
        self.get_logger().info("Thread de lecture série démarré")

    # ------------------------------------------------------------------
    # Connexion / Reconnexion
    # ------------------------------------------------------------------

    def _try_connect(self):
        """Tente d'ouvrir le port série. Retourne True si succès."""
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2.0)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            with self.ser_lock:
                self.ser = ser
            self.get_logger().info(f"Connecté à l'Arduino sur {self.port}")
            return True
        except Exception as e:
            self.get_logger().warn(f"Connexion impossible sur {self.port} : {e}")
            return False

    def _close_serial(self):
        """Ferme proprement le port série (thread-safe)."""
        with self.ser_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def _reconnect_loop(self):
        """Boucle de reconnexion : tente toutes les 3s jusqu'au succès."""
        self.get_logger().warn("Connexion série perdue — tentatives de reconnexion...")
        while self.running and rclpy.ok():
            time.sleep(3.0)
            if self._try_connect():
                return  # Reconnecté, on sort

    # ------------------------------------------------------------------
    # Envoi vers Arduino
    # ------------------------------------------------------------------

    def listener_callback(self, msg):
        """Envoi des commandes vers Arduino.
        Format TX: [0xFF, VITESSE, DIRECTION, CHECKSUM] = 4 octets
        """
        lin_x = msg.linear.x
        ang_z = msg.angular.z

        vitesse   = int(max(min(lin_x * 50 + 50, 100), 0))
        direction = int(max(min(ang_z * 50 + 50, 100), 0))
        checksum  = (vitesse + direction) & 0xFF
        trame     = struct.pack('BBBB', 0xFF, vitesse, direction, checksum)

      # self.get_logger().info(f"Envoi -> V:{vitesse} D:{direction} | Trame: {trame.hex(' ')}")
        self.get_logger().debug(f"Envoi -> V:{vitesse} D:{direction} | Trame: {trame.hex(' ')}")

        with self.ser_lock:
            ser = self.ser

        if ser is None:
            return  # Pas connecté, on ignore silencieusement

        try:
            ser.write(trame)
        except Exception as e:
            self.get_logger().error(f"Erreur d'écriture série : {e}")

    # ------------------------------------------------------------------
    # Lecture depuis Arduino
    # ------------------------------------------------------------------

    def read_serial_loop(self):
        """Thread de lecture continue. Gère la reconnexion automatique.
        Format RX attendu: [0xDD, IG_H, IG_L, ID_H, ID_L, Angle, CS] = 7 octets
        """
        buffer = bytearray()

        while self.running and rclpy.ok():

            # Pas de connexion active : on lance la reconnexion
            with self.ser_lock:
                ser = self.ser

            if ser is None:
                self._reconnect_loop()
                buffer.clear()  # Vider le buffer obsolète après reconnexion
                continue

            # Lecture normale
            try:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    buffer.extend(data)

                    while len(buffer) >= 7:
                        if buffer[0] == 0xDD:
                            self.parse_arduino_packet(buffer[:7])
                            buffer = buffer[7:]
                        else:
                            buffer.pop(0)  # Resync octet par octet
                else:
                    time.sleep(0.001)

            except (serial.SerialException, OSError) as e:
                # Errno 5 (I/O error) et Errno 6 (no such device) capturés ici
                self.get_logger().error(f"Port série perdu : {e}")
                self._close_serial()
                buffer.clear()
                # La prochaine itération déclenchera _reconnect_loop()

            except Exception as e:
                self.get_logger().error(f"Erreur lecture série inattendue : {e}")
                time.sleep(0.1)

    # ------------------------------------------------------------------
    # Parsing paquet Arduino
    # ------------------------------------------------------------------

    def parse_arduino_packet(self, packet):
        """Parse un paquet reçu de l'Arduino.

        Format: [0xDD, IG_H, IG_L, ID_H, ID_L, Angle, CS]
          - IR Gauche   = (packet[1] << 8) | packet[2]  → uint16, en mm
          - IR Droit    = (packet[3] << 8) | packet[4]  → uint16, en mm
          - Angle servo = packet[5]                     → uint8, en degrés
          - Checksum    = packet[6] == (IG_H+IG_L+ID_H+ID_L+Angle) & 0xFF
        """
        if len(packet) != 7:
            return

        ir_gauche     = (packet[1] << 8) | packet[2]
        ir_droit      = (packet[3] << 8) | packet[4]
        angle_servo   = packet[5]
        checksum_recu = packet[6]

        checksum_calc = (packet[1] + packet[2] + packet[3] + packet[4] + angle_servo) & 0xFF

        if checksum_calc != checksum_recu:
            self.get_logger().warn(
                f'Checksum invalide: calculé={checksum_calc} reçu={checksum_recu}'
            )
            return

        msg_ir_g = Float32()
        msg_ir_g.data = float(ir_gauche)
        self.pub_ir_gauche.publish(msg_ir_g)

        msg_ir_d = Float32()
        msg_ir_d.data = float(ir_droit)
        self.pub_ir_droit.publish(msg_ir_d)

        msg_servo = UInt8()
        msg_servo.data = angle_servo
        self.pub_servo_angle.publish(msg_servo)

        # Log optionnel (commenter en production si trop verbeux)
        # self.get_logger().info(
        #     f'RX <- IR_G={ir_gauche}mm IR_D={ir_droit}mm Servo={angle_servo}°'
        # )

    # ------------------------------------------------------------------
    # Nettoyage
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join(timeout=2.0)
        self._close_serial()
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
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
