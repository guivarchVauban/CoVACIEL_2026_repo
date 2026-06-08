#!/usr/bin/env python3
"""
ORCHESTRATEUR NODE - Covaciel
==============================
Machine d'état globale du robot.

Modes :
  0 → Stop / attente course_active
  1 → car_node pilote (autonome)
  2 → Demi-tour en cours (orchestrateur prend la main)
  3 → nav_node pilote (SLAM Eliot) + surveillance obstacles/demi-tour active

DEBUG = 1 → menu interactif terminal en parallèle du node ROS2
"""

import math
import threading
import time
import sys
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32, String, Float32


# ══════════════════════════════════════════════════════════════
#  DEBUG
# ══════════════════════════════════════════════════════════════

DEBUG = 1   # 0 = production | 1 = menu interactif terminal


# ══════════════════════════════════════════════════════════════
#  PARAMÈTRES
# ══════════════════════════════════════════════════════════════

CONTRESENS_THRESHOLD = 8

# IR / Ultrasons — placeholder
IR_SEUIL_RAW      = 212
IR_RAW_MAX_VALIDE = 950

# Demi-tour en 2 temps
DEMI_TOUR_CLEAR_LATERAL   = 0.30
DEMI_TOUR_CLEAR_FRONT     = 0.35
DEMI_TOUR_RECUL_VIT       = -0.12
DEMI_TOUR_RECUL_ANGULAR   =  0.5
DEMI_TOUR_RECUL_DUREE     =  1.0
DEMI_TOUR_AVANCE_VIT      =  0.15
DEMI_TOUR_AVANCE_ANGULAR  =  0.8
DEMI_TOUR_AVANCE_DUREE    =  1.2
DEMI_TOUR_TIMEOUT_GLOBAL  = 20.0
DEMI_TOUR_ATTENTE_IR_MAX  =  5.0

# Récupération blocage frontal (mode 3 uniquement)
STUCK_FRONT_THRESHOLD  = 0.20
STUCK_CONFIRM_FRAMES   = 5
STUCK_IR_WAIT_MAX      = 3.0
STUCK_RECUL_VITESSE    = -0.15
STUCK_RECUL_DUREE      =  0.8
STUCK_ROTATION_VITESSE =  0.5
STUCK_ROTATION_DUREE   =  1.0

TRANSITION_DELAY = 0.12
CMD_VEL_RATE     = 0.05

# Watchdog — délai max sans données avant alerte (secondes)
WATCHDOG_SCAN_MAX   = 3.0
WATCHDOG_CAMERA_MAX = 3.0
WATCHDOG_IR_MAX     = 5.0


# ══════════════════════════════════════════════════════════════
#  COULEURS TERMINAL
# ══════════════════════════════════════════════════════════════

class C:
    RESET  = '\033[0m'
    BOLD   = '\033[1m'
    RED    = '\033[91m'
    GREEN  = '\033[92m'
    YELLOW = '\033[93m'
    BLUE   = '\033[94m'
    CYAN   = '\033[96m'
    GRAY   = '\033[90m'
    WHITE  = '\033[97m'

def _c(color, text):
    return f"{color}{text}{C.RESET}"


# ══════════════════════════════════════════════════════════════
#  NODE
# ══════════════════════════════════════════════════════════════

class OrchestratorNode(Node):

    def __init__(self):
        super().__init__('orchestrateur_node')

        self.declare_parameter('mode_demarrage', 1)
        self._mode_demarrage = int(self.get_parameter('mode_demarrage').value)
        if self._mode_demarrage not in (1, 2, 3):
            self.get_logger().warning(
                f"mode_demarrage={self._mode_demarrage} invalide → forcé à 1"
            )
            self._mode_demarrage = 1

        label = {1: 'car_node', 2: 'TEST demi-tour', 3: 'nav_node SLAM'}[self._mode_demarrage]
        self.get_logger().info(
            f"=== ORCHESTRATEUR DÉMARRÉ === (mode_demarrage={self._mode_demarrage} → {label})"
        )

        # ── État ──
        self.robot_mode          = 0
        self.contresens_count    = 0
        self.demi_tour_en_cours  = False
        self.abort_demi_tour     = False
        self._recovery_en_cours  = False
        self._mode_retour        = 1
        self._shutdown_requested = False

        # ── Watchdog timestamps ──
        self._t_last_scan   = None
        self._t_last_camera = None
        self._t_last_ir     = None

        # ── Stats debug ──
        self._cmd_vel_count      = 0
        self._last_cmd_vel       = Twist()
        self._last_course_active = None

        # ── Données capteurs (thread-safe) ──
        self._lock            = threading.Lock()
        self._ir_gauche_raw   = 0.0
        self._ir_droit_raw    = 0.0
        self._bon_sens_val    = None
        self._stuck_frames    = 0
        self._last_scan_cte   = 0.0
        self._last_scan_avg_g = 4.0
        self._last_scan_avg_d = 4.0
        self._last_scan_front = 4.0

        # ── Publishers ──
        self.mode_pub    = self.create_publisher(Int32, '/robot_mode', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel',    10)

        # ── Subscribers ──
        self.create_subscription(Bool,      '/course_active',  self._cb_course_active, 10)
        self.create_subscription(Bool,      '/bon_sens',       self._cb_bon_sens,       10)
        self.create_subscription(String,    '/sens_demi_tour', self._cb_sens_demi_tour, 10)
        self.create_subscription(Float32,   '/ir_gauche',      self._cb_ir_gauche,      10)
        self.create_subscription(Float32,   '/ir_droit',       self._cb_ir_droit,       10)
        self.create_subscription(LaserScan, '/scan',           self._cb_scan,           10)

        self._publish_mode(0)

        if DEBUG:
            self._debug_thread = threading.Thread(target=self._debug_menu_loop, daemon=True)
            self._debug_thread.start()
        else:
            self.get_logger().info(
                "Mode 0 — Pour démarrer : "
                "ros2 topic pub --once /course_active std_msgs/msg/Bool '{data: true}'"
            )

    # ══════════════════════════════════════════
    #  CALLBACKS
    # ══════════════════════════════════════════

    def _cb_course_active(self, msg: Bool):
        self._last_course_active = msg.data
        if msg.data and self.robot_mode == 0:
            self.get_logger().info(f"GO → Mode {self._mode_demarrage}")
            self._mode_retour = 1
            self._transition_mode(self._mode_demarrage)
            if self._mode_demarrage == 2:
                threading.Thread(target=self._lancer_demi_tour, daemon=True).start()
        elif not msg.data and self.robot_mode != 0:
            self.get_logger().warning("STOP → Mode 0")
            self.abort_demi_tour    = True
            self._recovery_en_cours = False
            self._transition_mode(0)

    def _cb_bon_sens(self, msg: Bool):
        self._t_last_camera = time.time()
        with self._lock:
            self._bon_sens_val = msg.data

        if self.robot_mode not in (1,):
            self.contresens_count = 0
            return
        if self._recovery_en_cours or self.demi_tour_en_cours:
            return

        if not msg.data:
            self.contresens_count += 1
            self.get_logger().debug(
                f"Contresens {self.contresens_count}/{CONTRESENS_THRESHOLD}"
            )
            if self.contresens_count >= CONTRESENS_THRESHOLD:
                self.contresens_count = 0
                self.get_logger().warning("Contresens confirmé → Mode 2 (demi-tour)")
                self._mode_retour = 1
                self._transition_mode(2)
                threading.Thread(target=self._lancer_demi_tour, daemon=True).start()
        else:
            self.contresens_count = 0

    def _cb_sens_demi_tour(self, msg: String):
        pass  # topic reçu mais non utilisé pour l'instant

    def _cb_ir_gauche(self, msg: Float32):
        self._t_last_ir = time.time()
        with self._lock:
            self._ir_gauche_raw = msg.data

    def _cb_ir_droit(self, msg: Float32):
        self._t_last_ir = time.time()
        with self._lock:
            self._ir_droit_raw = msg.data

    def _cb_scan(self, msg: LaserScan):
        self._t_last_scan = time.time()
        ranges = msg.ranges
        if not ranges:
            return

        n     = len(ranges)
        mid   = n // 2
        clean = [
            r if (0.08 < r < 4.5 and not math.isinf(r)) else 4.0
            for r in ranges
        ]

        d_front = min(clean[-35:] + clean[:35])
        avg_d   = sum(clean[0:mid]) / mid
        avg_g   = sum(clean[mid:n]) / mid
        cte     = avg_g - avg_d

        with self._lock:
            self._last_scan_cte   = cte
            self._last_scan_avg_g = avg_g
            self._last_scan_avg_d = avg_d
            self._last_scan_front = d_front

        if self.robot_mode != 3:
            with self._lock:
                self._stuck_frames = 0
            return
        if self._recovery_en_cours or self.demi_tour_en_cours:
            return

        with self._lock:
            if d_front < STUCK_FRONT_THRESHOLD:
                self._stuck_frames += 1
            else:
                self._stuck_frames = 0
            frames = self._stuck_frames

        if frames >= STUCK_CONFIRM_FRAMES:
            self.get_logger().error(
                f"BLOCAGE ({d_front:.2f} m, {frames} frames) → récupération"
            )
            self._recovery_en_cours = True
            threading.Thread(target=self._lancer_recul_urgence, daemon=True).start()

    # ══════════════════════════════════════════
    #  DEBUG MENU
    # ══════════════════════════════════════════

    def _debug_menu_loop(self):
        """Menu interactif terminal — tourne dans un thread dédié."""
        time.sleep(1.5)  # Laisser ROS2 s'initialiser
        self._print_header()
        self._print_menu()

        while not self._shutdown_requested and rclpy.ok():
            if select.select([sys.stdin], [], [], 0.2)[0]:
                try:
                    raw = sys.stdin.readline().strip().lower()
                except Exception:
                    break
                if raw:
                    self._handle_debug_input(raw)
                    if not self._shutdown_requested:
                        self._print_menu()

    def _print_header(self):
        print("\n" + "═" * 58)
        print(_c(C.CYAN + C.BOLD, "  ORCHESTRATEUR COVACIEL — MODE DEBUG"))
        print("═" * 58)

    def _print_menu(self):
        now = time.time()

        # ── Watchdog nodes ──
        scan_ok = self._t_last_scan   is not None and (now - self._t_last_scan)   < WATCHDOG_SCAN_MAX
        cam_ok  = self._t_last_camera is not None and (now - self._t_last_camera) < WATCHDOG_CAMERA_MAX
        ir_ok   = self._t_last_ir     is not None and (now - self._t_last_ir)     < WATCHDOG_IR_MAX

        def node_line(ok, name, detail=""):
            dot   = _c(C.GREEN, "●") if ok else _c(C.RED, "●")
            state = _c(C.GREEN, "OK") if ok else _c(C.RED, "ABSENT/TIMEOUT")
            return f"  {dot} {name:<22} {state}  {_c(C.GRAY, detail)}"

        def age(t):
            return f"{now - t:.1f}s" if t else "jamais"

        # ── Données capteurs ──
        with self._lock:
            front = self._last_scan_front
            avg_g = self._last_scan_avg_g
            avg_d = self._last_scan_avg_d
            cte   = self._last_scan_cte
            ir_g  = self._ir_gauche_raw
            ir_d  = self._ir_droit_raw
            bon   = self._bon_sens_val

        mode_colors = {0: C.GRAY, 1: C.GREEN, 2: C.YELLOW, 3: C.BLUE}
        mode_labels = {0: "STOP/ATTENTE", 1: "car_node (PID)", 2: "DEMI-TOUR", 3: "nav_node SLAM"}
        mc = mode_colors.get(self.robot_mode, C.WHITE)
        ml = mode_labels.get(self.robot_mode, "?")

        cv     = self._last_cmd_vel
        cv_str = f"lin={cv.linear.x:+.2f}  ang={cv.angular.z:+.2f}  (#{self._cmd_vel_count} publiés)"

        if bon is None:
            bon_str = _c(C.GRAY, "pas de données")
        elif bon:
            bon_str = _c(C.GREEN, "BON SENS")
        else:
            bon_str = _c(C.RED, f"CONTRESENS  ({self.contresens_count}/{CONTRESENS_THRESHOLD})")

        if self._last_course_active is None:
            xbee_str = _c(C.GRAY, "pas de signal reçu")
        elif self._last_course_active:
            xbee_str = _c(C.GREEN, "GO")
        else:
            xbee_str = _c(C.YELLOW, "STOP")

        front_color = C.RED if front < 0.35 else (C.YELLOW if front < 0.8 else C.WHITE)

        print("\n" + "─" * 58)

        # État global
        print(_c(C.BOLD, "  ÉTAT"))
        print(f"  Mode actif    : {_c(mc + C.BOLD, f'[{self.robot_mode}] {ml}')}")
        print(f"  XBee          : {xbee_str}")
        print(f"  Demi-tour     : {_c(C.YELLOW, 'EN COURS') if self.demi_tour_en_cours else 'non'}")
        print(f"  Récupération  : {_c(C.YELLOW, 'EN COURS') if self._recovery_en_cours else 'non'}")

        # Nodes
        print(_c(C.BOLD, "\n  NODES"))
        print(node_line(scan_ok, "LiDAR /scan",      f"dernier: {age(self._t_last_scan)}"))
        print(node_line(cam_ok,  "Caméra /bon_sens",  f"dernier: {age(self._t_last_camera)}"))
        print(node_line(ir_ok,   "IR /ir_gauche+d",   f"dernier: {age(self._t_last_ir)}"))

        # Alertes nodes manquants
        if not scan_ok:
            print(_c(C.RED,    "  !! LiDAR absent — vérifier : screen -r lidar"))
        if not cam_ok:
            print(_c(C.RED,    "  !! Caméra absente — vérifier : screen -r nodecamera | USB ?"))
        if not ir_ok:
            print(_c(C.YELLOW, "  !! IR absent — placeholder actif (normal si pas de capteurs)"))

        # LiDAR
        print(_c(C.BOLD, "\n  LIDAR"))
        print(f"  Front : {_c(front_color, f'{front:.2f} m')}   "
              f"G: {avg_g:.2f} m   D: {avg_d:.2f} m   CTE: {cte:+.3f}")

        # Capteurs
        print(_c(C.BOLD, "\n  CAPTEURS"))
        print(f"  IR gauche raw : {ir_g:.0f}   IR droit raw : {ir_d:.0f}  "
              f"{_c(C.GRAY, '(placeholder ultrasons)')}")
        print(f"  Caméra        : {bon_str}")

        # cmd_vel
        print(_c(C.BOLD, "\n  CMD_VEL"))
        print(f"  {cv_str}")

        # Menu
        print(_c(C.BOLD, "\n  COMMANDES"))
        print("  [0] Mode 0 STOP          [1] Mode 1 car_node")
        print("  [2] Lancer demi-tour     [3] Mode 3 nav_node")
        print("  [g] Simuler GO (XBee)    [s] Simuler STOP (XBee)")
        print("  [r] Rafraîchir           [q] Arrêt propre")
        print("─" * 58)
        print("  > ", end='', flush=True)

    def _handle_debug_input(self, cmd: str):
        print()

        if cmd == '0':
            print(_c(C.YELLOW, "  → Forçage Mode 0 (STOP)"))
            self.abort_demi_tour    = True
            self._recovery_en_cours = False
            self._transition_mode(0)

        elif cmd == '1':
            print(_c(C.GREEN, "  → Forçage Mode 1 (car_node)"))
            self._mode_retour = 1
            self._transition_mode(1)

        elif cmd == '2':
            if self.demi_tour_en_cours:
                print(_c(C.RED, "  !! Demi-tour déjà en cours !"))
            else:
                print(_c(C.YELLOW, "  → Lancement demi-tour (Mode 2)"))
                self._mode_retour = 1
                self._transition_mode(2)
                threading.Thread(target=self._lancer_demi_tour, daemon=True).start()

        elif cmd == '3':
            print(_c(C.BLUE, "  → Forçage Mode 3 (nav_node SLAM)"))
            self._transition_mode(3)

        elif cmd == 'g':
            print(_c(C.GREEN, "  → Simulation GO"))
            msg = Bool(); msg.data = True
            self._cb_course_active(msg)

        elif cmd == 's':
            print(_c(C.YELLOW, "  → Simulation STOP"))
            msg = Bool(); msg.data = False
            self._cb_course_active(msg)

        elif cmd == 'r':
            pass  # _print_menu() appelé juste après dans la boucle

        elif cmd == 'q':
            print(_c(C.RED, "  → Arrêt propre..."))
            self._shutdown_requested = True
            self._stop_robot()
            self._publish_mode(0)
            rclpy.shutdown()

        else:
            print(_c(C.GRAY, f"  Commande inconnue : '{cmd}'"))

    # ══════════════════════════════════════════
    #  DEMI-TOUR
    # ══════════════════════════════════════════

    def _lancer_demi_tour(self):
        self.demi_tour_en_cours = True
        self.abort_demi_tour    = False
        t_global = time.time()

        self.get_logger().info("DEMI-TOUR 2 temps — démarrage")

        while True:
            if time.time() - t_global > DEMI_TOUR_TIMEOUT_GLOBAL:
                self.get_logger().error("Demi-tour timeout global → Mode 0")
                self._transition_mode(0)
                self.demi_tour_en_cours = False
                return
            if self.abort_demi_tour:
                self.get_logger().warning("Demi-tour interrompu (STOP)")
                self.demi_tour_en_cours = False
                return

            # Phase 0 : clearance LiDAR
            self.get_logger().info("  Phase 0 : vérif clearance corridor")
            while True:
                with self._lock:
                    avg_g = self._last_scan_avg_g
                    avg_d = self._last_scan_avg_d
                if avg_g >= DEMI_TOUR_CLEAR_LATERAL and avg_d >= DEMI_TOUR_CLEAR_LATERAL:
                    break
                if self.abort_demi_tour or time.time() - t_global > DEMI_TOUR_TIMEOUT_GLOBAL:
                    self.get_logger().warning("  Clearance impossible → on tente quand même")
                    break
                self.get_logger().warning(
                    f"  Corridor étroit G={avg_g:.2f} D={avg_d:.2f} — attente..."
                )
                time.sleep(0.2)

            # Phase 1 : vérif obstacle arrière + recul braqué
            self.get_logger().info("  Phase 1 : vérif obstacle arrière")
            t_ir = time.time()
            while self._obstacle_arriere():
                if self.abort_demi_tour or time.time() - t_global > DEMI_TOUR_TIMEOUT_GLOBAL:
                    self.get_logger().error("  Arrière bloqué → Mode 0")
                    self._transition_mode(0)
                    self.demi_tour_en_cours = False
                    return
                if time.time() - t_ir > DEMI_TOUR_ATTENTE_IR_MAX:
                    self.get_logger().error("  Arrière persistant → Mode 0")
                    self._transition_mode(0)
                    self.demi_tour_en_cours = False
                    return
                self.get_logger().warning("  Arrière occupé — attente...")
                time.sleep(0.2)

            self.get_logger().info("  Phase 1 : recul braqué")
            cmd_recul = Twist()
            cmd_recul.linear.x  = DEMI_TOUR_RECUL_VIT
            cmd_recul.angular.z = DEMI_TOUR_RECUL_ANGULAR
            t_end    = time.time() + DEMI_TOUR_RECUL_DUREE
            recul_ok = True
            while time.time() < t_end:
                if self.abort_demi_tour:
                    self._stop_robot(); self.demi_tour_en_cours = False; return
                if self._obstacle_arriere():
                    self.get_logger().warning("  Obstacle arrière pendant recul → stop")
                    self._stop_robot(); recul_ok = False; break
                self.cmd_vel_pub.publish(cmd_recul)
                self._last_cmd_vel = cmd_recul
                self._cmd_vel_count += 1
                time.sleep(CMD_VEL_RATE)
            self._stop_robot()
            time.sleep(0.15)

            if not recul_ok:
                self.get_logger().warning("  Recul interrompu — retry")
                time.sleep(0.3); continue

            # Phase 2 : vérif frontal + avance braquée
            if self.abort_demi_tour:
                self.demi_tour_en_cours = False; return

            with self._lock:
                front = self._last_scan_front
            if front < DEMI_TOUR_CLEAR_FRONT:
                self.get_logger().warning(f"  Frontal trop proche ({front:.2f} m) — retry")
                time.sleep(0.2); continue

            self.get_logger().info(f"  Phase 2 : avance braquée (front={front:.2f} m)")
            cmd_avance = Twist()
            cmd_avance.linear.x  = DEMI_TOUR_AVANCE_VIT
            cmd_avance.angular.z = -DEMI_TOUR_AVANCE_ANGULAR
            t_end     = time.time() + DEMI_TOUR_AVANCE_DUREE
            avance_ok = True
            while time.time() < t_end:
                if self.abort_demi_tour:
                    self._stop_robot(); self.demi_tour_en_cours = False; return
                with self._lock:
                    front_now = self._last_scan_front
                if front_now < DEMI_TOUR_CLEAR_FRONT:
                    self.get_logger().warning(f"  Obstacle frontal ({front_now:.2f} m) → stop")
                    self._stop_robot(); avance_ok = False; break
                self.cmd_vel_pub.publish(cmd_avance)
                self._last_cmd_vel = cmd_avance
                self._cmd_vel_count += 1
                time.sleep(CMD_VEL_RATE)
            self._stop_robot()
            time.sleep(0.1)

            if not avance_ok:
                self.get_logger().warning("  Avance interrompue — retry")
                time.sleep(0.3); continue

            self.get_logger().info(f"Demi-tour OK → Mode {self._mode_retour}")
            self.demi_tour_en_cours = False
            self._transition_mode(self._mode_retour)
            return

    # ══════════════════════════════════════════
    #  RÉCUPÉRATION BLOCAGE
    # ══════════════════════════════════════════

    def _lancer_recul_urgence(self):
        self._stop_robot()
        self._publish_mode(0)
        time.sleep(0.05)

        with self._lock:
            cte = self._last_scan_cte

        signe_recul = -1.0 if cte > 0 else 1.0
        self.get_logger().warning(f"[RECUL] CTE={cte:.2f} → signe={signe_recul:+.1f}")

        arriere_libre = False
        t_start       = time.time()
        while time.time() - t_start < STUCK_IR_WAIT_MAX:
            if not self._recovery_en_cours:
                self._recovery_cleanup(); return
            if not self._obstacle_arriere():
                arriere_libre = True; break
            self.get_logger().warning("[RECUL] Arrière occupé — attente...")
            time.sleep(0.2)

        if arriere_libre:
            cmd = Twist()
            cmd.linear.x  = STUCK_RECUL_VITESSE
            cmd.angular.z = signe_recul * STUCK_ROTATION_VITESSE
            self._send_cmd_vel_timed(cmd, STUCK_RECUL_DUREE,
                                     stop_check=lambda: not self._recovery_en_cours)
        else:
            self.get_logger().warning("[RECUL] Arrière bloqué → rotation sur place")
            cmd = Twist()
            cmd.angular.z = signe_recul * STUCK_ROTATION_VITESSE
            self._send_cmd_vel_timed(cmd, STUCK_ROTATION_DUREE,
                                     stop_check=lambda: not self._recovery_en_cours)

        self._stop_robot()
        time.sleep(0.1)
        self._recovery_cleanup()

        if not self._recovery_en_cours:
            return

        self.get_logger().info("[RECUL] Fin → reprise Mode 3")
        time.sleep(TRANSITION_DELAY)
        self._transition_mode(3)

    def _recovery_cleanup(self):
        with self._lock:
            self._stuck_frames = 0
        self._recovery_en_cours = False

    # ══════════════════════════════════════════
    #  OBSTACLE ARRIÈRE (placeholder ultrasons)
    # ══════════════════════════════════════════

    def _obstacle_arriere(self) -> bool:
        """Placeholder — ultrasons à câbler ici."""
        return False

    def _distance_arriere_mm(self) -> float:
        """Placeholder — retourne 9999 jusqu'aux ultrasons."""
        return 9999.0

    # ══════════════════════════════════════════
    #  UTILITAIRES
    # ══════════════════════════════════════════

    def _transition_mode(self, new_mode: int):
        self.get_logger().info(f"[TRANSITION] {self.robot_mode} → {new_mode}")
        self._stop_robot()
        time.sleep(0.05)
        self._publish_mode(0)
        time.sleep(TRANSITION_DELAY)
        self.robot_mode = new_mode
        self._publish_mode(new_mode)
        self.get_logger().info(f"[MODE {new_mode}] Actif")

    def _publish_mode(self, mode: int):
        msg      = Int32()
        msg.data = mode
        self.mode_pub.publish(msg)

    def _stop_robot(self):
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
        self._last_cmd_vel = stop

    def _send_cmd_vel_timed(self, cmd: Twist, duration: float, stop_check=None):
        t_end = time.time() + duration
        while time.time() < t_end:
            if stop_check and stop_check():
                break
            self.cmd_vel_pub.publish(cmd)
            self._last_cmd_vel = cmd
            self._cmd_vel_count += 1
            time.sleep(CMD_VEL_RATE)
        self._stop_robot()


# ══════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════

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
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
