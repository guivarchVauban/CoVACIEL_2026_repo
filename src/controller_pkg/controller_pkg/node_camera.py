import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool, String
import cv2
import numpy as np
import threading
import time


# --- PLAGES HSV ---
VERT_BAS  = np.array([45,  100,  60])
VERT_HAUT = np.array([75, 255, 255])

ROUGE_BAS_1  = np.array([0,   140, 80])
ROUGE_HAUT_1 = np.array([8,  255, 255])
ROUGE_BAS_2  = np.array([172, 140, 80])
ROUGE_HAUT_2 = np.array([180, 255, 255])

# Seuil minimum de pixels detectes pour valider une couleur
SEUIL_PIXELS = 500

# Angles servo
ANGLE_DROITE = 135
ANGLE_GAUCHE = 45
ANGLE_CENTRE = 90
TOLERANCE_ANGLE = 5

# Ordre de scan des index camera (video0 en priorite car mappe dans Docker)
CAMERA_INDICES = [0, 1, 2, 4, 6, 8, 10]


class NodeCamera(Node):
    def __init__(self):
        super().__init__('node_camera')

        # --- Parametres ---
        self.declare_parameter('camera_index', -1)  # -1 = scan automatique
        self.camera_index_param = self.get_parameter('camera_index').value

        # --- Etat interne ---
        self.angle_servo_actuel   = ANGLE_CENTRE
        self.vert_detecte_droite  = False
        self.rouge_detecte_gauche = False
        self.cap  = None
        self.frame = None
        self.lock  = threading.Lock()

        # --- Subscriber /servo_angle ---
        self.sub_servo = self.create_subscription(
            UInt8, 'servo_angle', self.callback_servo, 10)

        # --- Publishers ---
        self.pub_sens     = self.create_publisher(Bool,   'bon_sens', 10)
        self.pub_sens_str = self.create_publisher(String, 'sens_circulation', 10)

        # --- Thread de capture (gere aussi la reconnexion) ---
        self.running = True
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        # --- Timer de traitement couleur (10Hz) ---
        self.create_timer(0.1, self.traitement_couleur)

        self.get_logger().info("Node camera demarre — detection rouge/vert active")

    # ------------------------------------------------------------------
    # OUVERTURE CAMERA (scan agressif inspire du script Flask)
    # ------------------------------------------------------------------

    def get_camera_instance(self):
        """Teste les index camera dans l'ordre et retourne le premier qui fonctionne."""

        # Si un index est force via parametre ROS, on essaie uniquement celui-la
        if self.camera_index_param >= 0:
            indices = [self.camera_index_param]
        else:
            indices = CAMERA_INDICES

        for index in indices:
            self.get_logger().info(f"Test camera index {index}...")
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.get_logger().info(f"Camera trouvee sur l'index {index}")
                    return cap
                cap.release()
            else:
                cap.release()

        self.get_logger().warn("Aucune camera disponible — nouvelle tentative dans 3s")
        return None

    # ------------------------------------------------------------------
    # THREAD CAPTURE
    # ------------------------------------------------------------------

    def capture_loop(self):
        """Thread dedie : gere l'ouverture, la reconnexion et la capture."""
        while self.running and rclpy.ok():

            # Connexion / reconnexion
            if self.cap is None or not self.cap.isOpened():
                with self.lock:
                    self.frame = None  # On invalide le frame pendant la reconnexion
                self.cap = self.get_camera_instance()
                if self.cap is None:
                    time.sleep(3.0)
                    continue

            # Lecture
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.lock:
                    self.frame = frame
            else:
                self.get_logger().warn("Lecture camera echouee — reconnexion...")
                self.cap.release()
                self.cap = None
                time.sleep(1.0)

            time.sleep(0.033)  # ~30fps

    # ------------------------------------------------------------------
    # CALLBACKS & DETECTION
    # ------------------------------------------------------------------

    def callback_servo(self, msg):
        with self.lock:
            self.angle_servo_actuel = msg.data

    def detecter_vert(self, frame):
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masque = cv2.inRange(hsv, VERT_BAS, VERT_HAUT)
        return int(cv2.countNonZero(masque)) >= SEUIL_PIXELS

    def detecter_rouge(self, frame):
        hsv     = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masque1 = cv2.inRange(hsv, ROUGE_BAS_1, ROUGE_HAUT_1)
        masque2 = cv2.inRange(hsv, ROUGE_BAS_2, ROUGE_HAUT_2)
        masque  = cv2.bitwise_or(masque1, masque2)
        return int(cv2.countNonZero(masque)) >= SEUIL_PIXELS

    def traitement_couleur(self):
        """Timer callback 10Hz : analyse le frame selon l'angle du servo."""
        with self.lock:
            angle = self.angle_servo_actuel
            frame = self.frame.copy() if self.frame is not None else None

        if frame is None:
            return

        if abs(angle - ANGLE_DROITE) <= TOLERANCE_ANGLE:
            vert = self.detecter_vert(frame)
            with self.lock:
                self.vert_detecte_droite = vert
            self.get_logger().debug(f"[DROITE 30deg] Vert: {vert}")

        elif abs(angle - ANGLE_GAUCHE) <= TOLERANCE_ANGLE:
            rouge = self.detecter_rouge(frame)
            with self.lock:
                self.rouge_detecte_gauche = rouge
            self.get_logger().debug(f"[GAUCHE 150deg] Rouge: {rouge}")

        with self.lock:
            vert  = self.vert_detecte_droite
            rouge = self.rouge_detecte_gauche

        if vert and rouge:
            self.publier_sens(True, "BON_SENS")
        elif not vert and not rouge:
            self.publier_sens(None, "INCONNU")
        else:
            self.publier_sens(False, "SENS_INVERSE")

    def publier_sens(self, bon_sens, label):
        msg_str = String()
        msg_str.data = label
        self.pub_sens_str.publish(msg_str)

        if bon_sens is not None:
            msg_bool = Bool()
            msg_bool.data = bon_sens
            self.pub_sens.publish(msg_bool)

        self.get_logger().info(f"Sens circulation : {label}")

    # ------------------------------------------------------------------
    # NETTOYAGE
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join(timeout=2.0)
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera liberee")
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
