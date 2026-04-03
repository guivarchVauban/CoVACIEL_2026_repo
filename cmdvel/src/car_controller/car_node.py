import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32
import time, math, sys, select, termios, tty

class NavTunnelPIDNode(Node):
    def __init__(self):
        super().__init__('nav_tunnel_pid_node')
        self.get_logger().warning(">>> NAV CAPTAIN V3 : FILTRE PASSE-BAS + DIAGONALES + IR <<<")

        # --- GESTION DU MODE ---
        self._robot_mode = 0
        self.mode_publisher = self.create_publisher(Int32, '/robot_mode', 10)
        self.create_subscription(Int32, '/robot_mode', self._cb_mode, 10)

        # --- CAPTEURS INFRAROUGE (RECUL) ---
        self.ir_droit = 100.0
        self.ir_gauche = 100.0
        self.create_subscription(Float32, '/ir_droit', self._cb_ir_d, 10)
        self.create_subscription(Float32, '/ir_gauche', self._cb_ir_g, 10)
        self.STEER_AVOID_IR = 1.0 

        # --- FILTRE PASSE-BAS (LISSAGE) ---
        self.smooth_g = 1.0
        self.smooth_d = 1.0
        self.alpha = 0.2  # 0.2 = 20% nouvelle mesure, 80% mémoire (plus c'est bas, plus c'est fluide)

        # --- PARAMÈTRES PID ---
        self.tau_p = 1.5  # Légèrement baissé pour la stabilité
        self.tau_d = 3.5  # Augmenté pour amortir les oscillations
        self.prev_cte = 0.0

        # --- VITESSES ---
        self.SPEED_FAST = 0.25
        self.SPEED_MED  = 0.2
        self.SPEED_SLOW = 0.1
        self.SPEED_REVERSE = -0.15

        # --- ÉTATS ---
        self.current_state = "AVANCE"
        self.state_timer = 0.0
        self.reverse_steer = 0.0
        self.drift_until = 0.0
        self.last_drift_time = 0.0
        self.drift_duration = 0.3

        self.last_log_time = 0.0
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)

    # --- CALLBACKS IR ---
    def _cb_ir_d(self, msg):
        self.ir_droit = msg.data if msg.data > 10.0 else 5.0
    def _cb_ir_g(self, msg):
        self.ir_gauche = msg.data if msg.data > 10.0 else 5.0

    def toggle_mode(self):
        new_mode = 0 if self._robot_mode == 1 else 1
        msg = Int32(); msg.data = new_mode
        self.mode_publisher.publish(msg)
        self._robot_mode = new_mode
        if new_mode == 0: self.stop_car()
        self.get_logger().info(f"MODE: {'ON' if new_mode==1 else 'OFF'}")

    def _cb_mode(self, msg):
        self._robot_mode = msg.data
        if self._robot_mode != 1: self.stop_car()

    def stop_car(self):
        msg = Twist()
        msg.linear.x, msg.angular.z = 0.0, 0.0
        self.publisher.publish(msg)

    def listener_callback(self, msg):
        if self._robot_mode != 1: return

        ranges = msg.ranges
        if not ranges: return
        now = time.time()
        n = len(ranges)
        
        # Nettoyage LiDAR
        clean = [r if (0.08 < r < 4.5 and not math.isinf(r)) else 4.0 for r in ranges]
        
        # --- 1. CALCUL BRUT SUR DIAGONALES (15%-25% et 75%-85%) ---
        raw_d = sum(clean[int(n*0.15):int(n*0.25)]) / (int(n*0.25) - int(n*0.15))
        raw_g = sum(clean[int(n*0.75):int(n*0.85)]) / (int(n*0.85) - int(n*0.75))
        d_front = min(clean[-35:] + clean[:35])

        # --- 2. FILTRAGE PASSE-BAS (LISSAGE TEMPOREL) ---
        self.smooth_d = (self.alpha * raw_d) + ((1.0 - self.alpha) * self.smooth_d)
        self.smooth_g = (self.alpha * raw_g) + ((1.0 - self.alpha) * self.smooth_g)

        # --- 3. CALCUL PID ---
        cte = (self.smooth_g - self.smooth_d)
        diff_cte = cte - self.prev_cte
        self.prev_cte = cte

        cmd = Twist()
        mode_label = "AUTO"

        # --- ÉTAT : AVANCE ---
        if self.current_state == "AVANCE":
            if d_front < 0.15:
                self.current_state = "STOP"
                self.state_timer = now + 0.5
                self.reverse_steer = -1.0 if cte > 0 else 1.0
                self.stop_car()
                return

            # Boost P si obstacle proche devant
            current_p = self.tau_p
            if d_front < 1.0:
                current_p = self.tau_p * 1.3 

            steer = (current_p * cte) + (self.tau_d * diff_cte)
            
            # Condition Drift (plus sélective : cte > 1.4)
            trigger_drift = (0.3 < d_front < 1.1) and (abs(cte) > 1.20) and (now - self.last_drift_time > 1.25)

            if trigger_drift:
                self.drift_until = now + self.drift_duration
                self.last_drift_time = now

            if now < self.drift_until:
                cmd.linear.x, limit_angle = 0.8, 1.9
                mode_label = "DRIFT"
            else:
                if d_front > 2.5: cmd.linear.x, limit_angle = self.SPEED_FAST, 1.1
                elif d_front > 1.3: cmd.linear.x, limit_angle = self.SPEED_MED, 1.6
                elif d_front > 0.4: cmd.linear.x, limit_angle = self.SPEED_SLOW, 2.0
                else: cmd.linear.x, limit_angle = 0.08, 2.5
            
            cmd.angular.z = float(max(min(steer, limit_angle), -limit_angle))

        # --- ÉTAT : STOP ---
        elif self.current_state == "STOP":
            cmd.linear.x, cmd.angular.z = 0.0, 0.0
            mode_label = "STOP"
            if now > self.state_timer:
                self.current_state = "RECUL"
                self.state_timer = now + 1.2

        # --- ÉTAT : RECUL ---
        elif self.current_state == "RECUL":
            cmd.linear.x = self.SPEED_REVERSE
            mode_label = "RECUL"
            
            # Priorité Infrarouge pour ne pas taper derrière
            if self.ir_droit < 18.0:
                cmd.angular.z = self.STEER_AVOID_IR
                mode_label = "RECUL_IR_D"
            elif self.ir_gauche < 18.0:
                cmd.angular.z = -self.STEER_AVOID_IR
                mode_label = "RECUL_IR_G"
            else:
                cmd.angular.z = self.reverse_steer

            if now > self.state_timer and d_front > 0.4:
                self.current_state = "AVANCE"
                self.prev_cte = 0.0

        self.publisher.publish(cmd)

        # --- LOGS ---
        if now - self.last_log_time > 0.2:
            self.get_logger().info(
                f"[{mode_label}] F:{d_front:.2f}m | G_Sm:{self.smooth_g:.2f}m | D_Sm:{self.smooth_d:.2f}m | Steer:{cmd.angular.z:.2f}"
            )
            self.last_log_time = now

# --- CLAVIER ---
def is_key_pressed(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
    key = sys.stdin.read(1) if rlist else None
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = NavTunnelPIDNode()
    settings = termios.tcgetattr(sys.stdin)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = is_key_pressed(settings)
            if key == ' ': node.toggle_mode()
            elif key in ['q', 'Q', 's', 'S']: break
    except KeyboardInterrupt: pass
    finally:
        node.stop_car()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()