#!/usr/bin/env python3
"""
ORCHESTRATEUR NODE - Covaciel
==============================
Cerveau central du robot. Gère la machine d'état globale.
 
Modes :
  0 → Arrêt / Attente /course_active=True
  1 → Tour 1  : PID LiDAR (nav_tunnel_node actif)
  2 → Demi-tour sécurisé (vérif IR + programme selon /sens_demi_tour)
  3 → Tour 2  : Stack SLAM coéquipier
 
Topics consommés :
  /course_active    (Bool)   — démarrage/arrêt (XBee plus tard, manuel pour l'instant)
  /bon_sens         (Bool)   — camera_node : True=bon sens, False=contresens
  /sens_demi_tour   (String) — camera_node : 'gauche' ou 'droite'
  /ir_gauche        (UInt16) — node_controller : valeur brute Sharp GP2Y0A21
  /ir_droit         (UInt16) — node_controller : valeur brute Sharp GP2Y0A21
 
Topics produits :
  /robot_mode       (Int32)  — source de vérité pour toutes les nodes
  /cmd_vel          (Twist)  — stops de transition + phases demi-tour
"""
 
import threading
import time
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, String, UInt16
 
 
# ══════════════════════════════════════════════════════════════
#  PARAMÈTRES
# ══════════════════════════════════════════════════════════════
 
# Détection contresens
CONTRESENS_THRESHOLD = 8     # frames False consécutifs → ~800ms à 10Hz caméra
 
# Seuil obstacle IR (Sharp GP2Y0A21 : distance_mm = 48000 / (raw - 20))
# raw=212 → ~250mm — en-dessous de cette distance = obstacle détecté
IR_SEUIL_RAW       = 212
IR_RAW_MAX_VALIDE  = 950     # au-dessus = saturation (objet < ~50mm)
 
# Demi-tour
DEMI_TOUR_DEFAULT          = 'gauche'  # fallback si caméra muette
DEMI_TOUR_RECUL_VITESSE    = -0.15
DEMI_TOUR_RECUL_DUREE      = 0.8      # secondes
DEMI_TOUR_ROTATION_VITESSE = 0.6      # angular.z
DEMI_TOUR_ROTATION_DUREE   = 2.2      # secondes — À CALIBRER SUR PISTE
DEMI_TOUR_ATTENTE_IR_MAX   = 5.0      # timeout obstacle arrière persistant
 
# Transitions
TRANSITION_DELAY = 0.12   # secondes de propagation entre mode=0 et nouveau mode
CMD_VEL_RATE     = 0.05   # intervalle de republication cmd_vel pendant demi-tour
 
 
# ══════════════════════════════════════════════════════════════
#  NODE
# ══════════════════════════════════════════════════════════════
 
class OrchestratorNode(Node):
 
    def __init__(self):
        super().__init__('orchestrateur_node')
        self.get_logger().info("=== ORCHESTRATEUR COVACIEL DÉMARRÉ ===")
 
        # État machine d'état
        self.robot_mode         = 0
        self.contresens_count   = 0
        self.demi_tour_en_cours = False
        self.abort_demi_tour    = False
 
        # Données capteurs (partagées avec thread demi-tour)
        self._lock           = threading.Lock()
        self._ir_gauche_raw  = 0
        self._ir_droit_raw   = 0
        self._sens_demi_tour = None   # 'gauche' | 'droite' | None
 
        # Publishers
        self.mode_pub    = self.create_publisher(Int32, '/robot_mode', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel',    10)
 
        # Subscribers
        self.create_subscription(Bool,   '/course_active',  self.cb_course_active,  10)
        self.create_subscription(Bool,   '/bon_sens',       self.cb_bon_sens,        10)
        self.create_subscription(String, '/sens_demi_tour', self.cb_sens_demi_tour,  10)
        self.create_subscription(UInt16, '/ir_gauche',      self.cb_ir_gauche,       10)
        self.create_subscription(UInt16, '/ir_droit',       self.cb_ir_droit,        10)
 
        self._publish_mode(0)
        self.get_logger().info(
            "Mode 0 — Pour démarrer : "
            "ros2 topic pub --once /course_active std_msgs/msg/Bool '{data: true}'"
        )
 
 
    # ══════════════════════════════════════════
    #  CALLBACKS
    # ══════════════════════════════════════════
 
    def cb_course_active(self, msg: Bool):
        if msg.data and self.robot_mode == 0:
            self.get_logger().info("GO → Mode 1 (Tour 1 PID LiDAR)")
            self._transition_mode(1)
 
        elif not msg.data and self.robot_mode != 0:
            self.get_logger().warning("STOP → Mode 0")
            self.abort_demi_tour = True
            self._transition_mode(0)
 
    def cb_bon_sens(self, msg: Bool):
        """Comptage des frames de contresens. Déclenchement mode 2 au seuil."""
        if self.robot_mode != 1:
            self.contresens_count = 0
            return
 
        if not msg.data:
            self.contresens_count += 1
            self.get_logger().debug(
                f"Contresens frame {self.contresens_count}/{CONTRESENS_THRESHOLD}"
            )
            if self.contresens_count >= CONTRESENS_THRESHOLD:
                self.contresens_count = 0
                self.get_logger().warning("Contresens confirmé → Mode 2 (Demi-tour)")
                self._transition_mode(2)
                threading.Thread(target=self._lancer_demi_tour, daemon=True).start()
        else:
            self.contresens_count = 0
 
    def cb_sens_demi_tour(self, msg: String):
        with self._lock:
            self._sens_demi_tour = msg.data
 
    def cb_ir_gauche(self, msg: UInt16):
        with self._lock:
            self._ir_gauche_raw = msg.data
 
    def cb_ir_droit(self, msg: UInt16):
        with self._lock:
            self._ir_droit_raw = msg.data
 
 
    # ══════════════════════════════════════════
    #  DEMI-TOUR — thread daemon
    # ══════════════════════════════════════════
 
    def _lancer_demi_tour(self):
        """
        3 phases : attente IR libre → recul bragué → rotation
        Tourne en thread daemon : le spin() continue de recevoir STOP etc.
        """
        self.demi_tour_en_cours = True
        self.abort_demi_tour    = False
 
        with self._lock:
            direction = self._sens_demi_tour
        direction = direction or DEMI_TOUR_DEFAULT
        signe = 1.0 if direction == 'gauche' else -1.0
 
        self.get_logger().info(
            f"DEMI-TOUR {direction.upper()} — Phase 0 : vérif arrière"
        )
 
        # Phase 0 : attendre que l'arrière soit libre
        t_start = time.time()
        while self._obstacle_arriere():
            if self.abort_demi_tour:
                self.get_logger().warning("Demi-tour interrompu (STOP)")
                self.demi_tour_en_cours = False
                return
            if time.time() - t_start > DEMI_TOUR_ATTENTE_IR_MAX:
                self.get_logger().error(
                    "Obstacle arrière persistant — annulé → Mode 0"
                )
                self._transition_mode(0)
                self.demi_tour_en_cours = False
                return
            self.get_logger().warning(
                f"Obstacle arrière ({self._distance_arriere_mm():.0f} mm) — attente..."
            )
            time.sleep(0.2)
 
        # Phase 1 : recul bragué
        if self.abort_demi_tour:
            self.demi_tour_en_cours = False
            return
 
        self.get_logger().info("Phase 1 : recul")
        cmd_recul = Twist()
        cmd_recul.linear.x  = DEMI_TOUR_RECUL_VITESSE
        cmd_recul.angular.z = signe * DEMI_TOUR_ROTATION_VITESSE * 0.4
        self._send_cmd_vel_timed(cmd_recul, DEMI_TOUR_RECUL_DUREE)
 
        self._stop_robot()
        time.sleep(0.15)
 
        # Phase 2 : rotation sur place
        if self.abort_demi_tour:
            self.demi_tour_en_cours = False
            return
 
        self.get_logger().info(f"Phase 2 : rotation {direction}")
        cmd_rot = Twist()
        cmd_rot.angular.z = signe * DEMI_TOUR_ROTATION_VITESSE
        self._send_cmd_vel_timed(cmd_rot, DEMI_TOUR_ROTATION_DUREE)
 
        self._stop_robot()
        time.sleep(0.1)
 
        if self.abort_demi_tour:
            self.demi_tour_en_cours = False
            return
 
        self.get_logger().info("Demi-tour OK → Mode 3 (SLAM)")
        self.demi_tour_en_cours = False
        self._transition_mode(3)
 
 
    # ══════════════════════════════════════════
    #  DÉTECTION IR
    # ══════════════════════════════════════════
 
    def _ir_obstacle(self, raw: int) -> bool:
        """True si obstacle détecté (< ~250mm)."""
        if raw <= 20 or raw > IR_RAW_MAX_VALIDE:
            return False  # lecture invalide → on laisse passer
        return raw > IR_SEUIL_RAW
 
    def _obstacle_arriere(self) -> bool:
        with self._lock:
            g, d = self._ir_gauche_raw, self._ir_droit_raw
        return self._ir_obstacle(g) or self._ir_obstacle(d)
 
    def _distance_arriere_mm(self) -> float:
        """Distance minimale arrière en mm (pour les logs)."""
        with self._lock:
            g, d = self._ir_gauche_raw, self._ir_droit_raw
        def to_mm(r):
            if r <= 20 or r > IR_RAW_MAX_VALIDE:
                return 9999.0
            return 48000.0 / (r - 20)
        return min(to_mm(g), to_mm(d))
 
 
    # ══════════════════════════════════════════
    #  UTILITAIRES
    # ══════════════════════════════════════════
 
    def _transition_mode(self, new_mode: int):
        """
        Séquence de transition propre :
          1. Stop robot (cmd_vel passe encore car mode actif)
          2. Publie mode=0 (toutes les nodes se taisent)
          3. Attend TRANSITION_DELAY (propagation)
          4. Publie le nouveau mode
        """
        self.get_logger().info(f"[TRANSITION] {self.robot_mode} → {new_mode}")
        self._stop_robot()
        time.sleep(0.05)
        self._publish_mode(0)
        time.sleep(TRANSITION_DELAY)
        self.robot_mode = new_mode
        self._publish_mode(new_mode)
        self.get_logger().info(f"[MODE {new_mode}] Actif")
 
    def _publish_mode(self, mode: int):
        msg = Int32()
        msg.data = mode
        self.mode_pub.publish(msg)
 
    def _stop_robot(self):
        self.cmd_vel_pub.publish(Twist())
 
    def _send_cmd_vel_timed(self, cmd: Twist, duration: float):
        """Publie cmd en boucle pendant duration secondes, avec check abort."""
        t_end = time.time() + duration
        while time.time() < t_end:
            if self.abort_demi_tour:
                break
            self.cmd_vel_pub.publish(cmd)
            time.sleep(CMD_VEL_RATE)
        self._stop_robot()
 
 
# ══════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════
 
def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        node._publish_mode(0)
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 
