#!/usr/bin/env python3
"""
NODE XBEE - Covaciel
=====================
Lit le flux série du récepteur XBee et publie sur /course_active.

  $GO;  → True   (démarrage course)
  STOP  → False  (arrêt course)

Le flux peut arriver concaténé : '$GO;STOPSTOP$GO;' — le buffer
est scanné token par token, seul le DERNIER token reçu est publié
pour éviter les états intermédiaires parasites.

Topic produit :
  /course_active (Bool)

Paramètres ROS2 :
  port     (str) — défaut /dev/ttyXBee
  baudrate (int) — défaut 9600
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial


TOKEN_GO   = b'$GO;'
TOKEN_STOP = b'STOP'
# Longueur commune des deux tokens
TOKEN_LEN  = 4


class NodeXBee(Node):

    def __init__(self):
        super().__init__('node_xbee')

        self.declare_parameter('port',     '/dev/ttyXBee')
        self.declare_parameter('baudrate', 9600)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baudrate').value)

        self.ser         = None
        self.ser_lock    = threading.Lock()
        self.running     = True
        self._last_state = None   # Évite de republier le même état

        self.pub = self.create_publisher(Bool, '/course_active', 10)

        self._try_connect()

        self.read_thread = threading.Thread(
            target=self._read_loop, daemon=True
        )
        self.read_thread.start()
        self.get_logger().info(
            f"XBee démarré sur {self.port} @ {self.baud} baud"
        )

    # ──────────────────────────────────────────
    #  Connexion
    # ──────────────────────────────────────────

    def _try_connect(self) -> bool:
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(0.5)
            ser.reset_input_buffer()
            with self.ser_lock:
                self.ser = ser
            self.get_logger().info(f"XBee connecté sur {self.port}")
            return True
        except Exception as e:
            self.get_logger().warn(f"XBee non disponible sur {self.port} : {e}")
            return False

    def _close_serial(self):
        with self.ser_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def _reconnect_loop(self):
        self.get_logger().warn("XBee perdu — tentatives de reconnexion...")
        while self.running and rclpy.ok():
            time.sleep(3.0)
            if self._try_connect():
                return

    # ──────────────────────────────────────────
    #  Lecture série
    # ──────────────────────────────────────────

    def _read_loop(self):
        """Thread principal : lit le flux et publie les changements d'état."""
        buf = bytearray()

        while self.running and rclpy.ok():

            with self.ser_lock:
                ser = self.ser

            if ser is None:
                self._reconnect_loop()
                buf.clear()
                continue

            try:
                if ser.in_waiting > 0:
                    buf.extend(ser.read(ser.in_waiting))
                    self._parse_buffer(buf)
                else:
                    time.sleep(0.005)

            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"Port XBee perdu : {e}")
                self._close_serial()
                buf.clear()

            except Exception as e:
                self.get_logger().error(f"Erreur lecture XBee inattendue : {e}")
                time.sleep(0.1)

    # ──────────────────────────────────────────
    #  Parsing
    # ──────────────────────────────────────────

    def _parse_buffer(self, buf: bytearray):
        """
        Extrait tous les tokens du buffer dans l'ordre.
        Seul le dernier token trouvé est publié (évite les aller-retours
        parasites si plusieurs tokens arrivent en rafale).
        Supprime du buffer tout ce qui a été consommé.
        """
        last_state = None
        pos        = 0

        while pos <= len(buf) - TOKEN_LEN:
            chunk = bytes(buf[pos:pos + TOKEN_LEN])

            if chunk == TOKEN_GO:
                last_state = True
                pos += TOKEN_LEN

            elif chunk == TOKEN_STOP:
                last_state = False
                pos += TOKEN_LEN

            else:
                # Octet parasite — on avance d'un cran et on cherche le prochain token
                pos += 1

        # Conserver uniquement ce qui n'a pas pu être parsé (< TOKEN_LEN octets)
        del buf[:pos]

        if last_state is None:
            return  # Rien de complet trouvé cette fois

        # Publier uniquement si l'état change
        if last_state != self._last_state:
            self._last_state = last_state
            msg      = Bool()
            msg.data = last_state
            self.pub.publish(msg)
            label = "GO  → course active" if last_state else "STOP → course arrêtée"
            self.get_logger().info(f"[XBEE] {label}")

    # ──────────────────────────────────────────
    #  Nettoyage
    # ──────────────────────────────────────────

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join(timeout=2.0)
        self._close_serial()
        self.get_logger().info("XBee fermé")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NodeXBee()
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
