import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import cv2
import numpy as np
import threading
import time

# --- MODE DEBUG ---
DEBUG      = True
MJPEG_PORT = 8081

# --- PLAGES HSV ÉLARGIES (Spécial caméra à exposition automatique) ---
VERT_BAS  = np.array([35,  50,  40])  
VERT_HAUT = np.array([85, 255, 255])

ROUGE_BAS_1  = np.array([0,   70,  50])
ROUGE_HAUT_1 = np.array([10,  255, 255])
ROUGE_BAS_2  = np.array([170, 70,  50])
ROUGE_HAUT_2 = np.array([180, 255, 255])

SEUIL_PIXELS_CONFIDENCE = 1500

CAMERA_INDICES = [0, 1, 2, 4, 6, 8, 10]

# Noyau pour nettoyer le bruit (Fermeture des micros-trous dans les masques)
KERNEL = np.ones((5, 5), np.uint8)

# ------------------------------------------------------------------
# SERVEUR MJPEG (DEBUG)
# ------------------------------------------------------------------
if DEBUG:
    from http.server import BaseHTTPRequestHandler, HTTPServer

    _debug_frame_lock = threading.Lock()
    _debug_frame      = None

    class _MJPEGHandler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def do_GET(self):
            if self.path != '/':
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    with _debug_frame_lock:
                        frame = _debug_frame
                    if frame is None:
                        time.sleep(0.05)
                        continue
                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    data = jpeg.tobytes()
                    self.wfile.write(
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + data + b'\r\n'
                    )
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass


class NodeCamera(Node):
    def __init__(self):
        super().__init__('node_camera')

        self.declare_parameter('camera_index', -1)
        self.camera_index_param = self.get_parameter('camera_index').value

        self.cap   = None
        self.frame = None
        self.lock  = threading.Lock()

        self.dernier_etat_publie = None

        # Publishers
        self.pub_bon_sens = self.create_publisher(Bool,   '/bon_sens',     10)
        self.pub_debug    = self.create_publisher(String, 'debug_scores',  10)

        self.running = True
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        if DEBUG:
            server = HTTPServer(('0.0.0.0', MJPEG_PORT), _MJPEGHandler)
            threading.Thread(target=server.serve_forever, daemon=True).start()
            self.get_logger().info(f"[DEBUG] Stream MJPEG : http://localhost:{MJPEG_PORT}/")

        self.create_timer(0.1, self.traitement_couleur)

        self.get_logger().info("Node camera initialisé.")

    def get_camera_instance(self):
        indices = [self.camera_index_param] if self.camera_index_param >= 0 else CAMERA_INDICES
        for index in indices:
            self.get_logger().info(f"Test camera index {index}...")
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                
                ret, frame = cap.read()
                if ret and frame is not None:
                    return cap
                cap.release()
            else:
                cap.release()
        return None

    def capture_loop(self):
        while self.running and rclpy.ok():
            if self.cap is None or not self.cap.isOpened():
                with self.lock:
                    self.frame = None
                self.cap = self.get_camera_instance()
                if self.cap is None:
                    time.sleep(3.0)
                    continue

            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.lock:
                    self.frame = frame
            else:
                if self.cap is not None:
                    self.cap.release()
                self.cap = None
                time.sleep(1.0)

            time.sleep(0.033)

    def traitement_couleur(self):
        with self.lock:
            frame = self.frame.copy() if self.frame is not None else None

        if frame is None:
            return

        h, w = frame.shape[:2]
        frame_bas = frame[h // 2:, :]

        # Étape 1 : Atténuation du bruit capteur (essentiel sur image fixe)
        blur = cv2.GaussianBlur(frame_bas, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        masque_v  = cv2.inRange(hsv, VERT_BAS, VERT_HAUT)
        masque_r1 = cv2.inRange(hsv, ROUGE_BAS_1, ROUGE_HAUT_1)
        masque_r2 = cv2.inRange(hsv, ROUGE_BAS_2, ROUGE_HAUT_2)
        masque_r  = cv2.bitwise_or(masque_r1, masque_r2)

        # Étape 2 : Closing morphologique pour souder les zones de couleur fragmentées
        masque_v = cv2.morphologyEx(masque_v, cv2.MORPH_CLOSE, KERNEL)
        masque_r = cv2.morphologyEx(masque_r, cv2.MORPH_CLOSE, KERNEL)

        total_vert  = int(cv2.countNonZero(masque_v))
        total_rouge = int(cv2.countNonZero(masque_r))

        etat_actuel  = None
        label_status = "INCONNU"

        if total_rouge > SEUIL_PIXELS_CONFIDENCE and total_rouge > total_vert:
            etat_actuel  = False
            label_status = "CONTRESENS"
        elif total_vert > SEUIL_PIXELS_CONFIDENCE and total_vert > total_rouge:
            etat_actuel  = True
            label_status = "BON_SENS"

        # --- LOGIQUE DE PUBLICATION SYNC AVEC L'ORCHESTRATEUR ---
        if etat_actuel is not None:
            msg = Bool()
            msg.data = etat_actuel
            self.pub_bon_sens.publish(msg)
            
            if etat_actuel != self.dernier_etat_publie:
                self.get_logger().warn(f"CHANGEMENT D'ÉTAT : {label_status} ({etat_actuel})")
                self.dernier_etat_publie = etat_actuel
        else:
            # Si c'est INCONNU, on force un 'True' (BON_SENS) pour rassurer l'orchestrateur
            msg = Bool()
            msg.data = True
            self.pub_bon_sens.publish(msg)
            self.dernier_etat_publie = None

        if DEBUG:
            msg_debug = String()
            msg_debug.data = (
                f"V:{total_vert} R:{total_rouge} | "
                f"STATUS:{label_status} | PUB_ACTUELLE:{self.dernier_etat_publie}"
            )
            self.pub_debug.publish(msg_debug)

            debug_frame = frame.copy()
            cv2.line(debug_frame, (0, h // 2), (w, h // 2), (128, 128, 128), 1)
            couleur_text = (
                (0, 255, 0) if etat_actuel is True
                else ((0, 0, 255) if etat_actuel is False
                      else (0, 165, 255))
            )
            cv2.putText(debug_frame, f"ETAT: {label_status}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, couleur_text, 2)
            cv2.putText(debug_frame, f"V: {total_vert}",
                        (10, h // 2 + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(debug_frame, f"R: {total_rouge}",
                        (10, h // 2 + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            global _debug_frame
            with _debug_frame_lock:
                _debug_frame = debug_frame

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join(timeout=2.0)
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NodeCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
