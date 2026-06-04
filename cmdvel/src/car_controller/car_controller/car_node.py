import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32
import time, math, sys, select, termios, tty

class NavTunnelPIDNode(Node):
    def __init__(self):
        super().__init__('nav_tunnel_pid_node')
        self.get_logger().warning(">>> NAV CAPTAIN V3.6 : OVERTAKEeeeeeeee INTELLIGENT <<<")

        self._robot_mode = 0
        self.mode_publisher = self.create_publisher(Int32, '/robot_mode', 10)
        self.create_subscription(Int32, '/robot_mode', self._cb_mode, 10)

        self.ir_droit = 100.0
        self.ir_gauche = 100.0
        self.create_subscription(Float32, '/ir_droit', self._cb_ir_d, 10)
        self.create_subscription(Float32, '/ir_gauche', self._cb_ir_g, 10)
        self.STEER_AVOID_IR = 1.0

        self.smooth_g = 1.0
        self.smooth_d = 1.0
        self.alpha = 0.35

        self.tau_p = 0.8
        self.tau_d = 4.5
        self.prev_cte = 0.0

        self.SPEED_FAST = 0.5
        self.SPEED_MED  = 0.27
        self.SPEED_SLOW = 0.1   
        self.SPEED_REVERSE = -0.1

        self.current_state = "AVANCE"
        self.state_timer = 0.0
        self.COOLDOWN_DURATION = 1.2
        
        # --- REPRISE APRÈS CRASH (ÉVASION) ---
        self.direction_evasion = 0.0        # Mémoire du côté libre lors du crash
        self.evasion_until = 0.0            # Timer pour maintenir le coup de volant
        self.EVASION_DURATION = 0.0        # 🔥 Durée forcée du coup de volant en secondes (Ajustable !)
        
        self.reverse_steer = 0.0
        self.drift_until = 0.0
        self.last_drift_time = 0.0
        self.drift_duration = 0.1
        self.drift_cooldown_until = 0.0

        # --- DONUT MODE ---
        self.donut_mode = False
        self.DONUT_SPEED = 0.67
        self.DONUT_STEER = 2.0
        self.DONUT_DIRECTION = 1.0

        # --- OVERTAKE PARAMÈTRES MODIFIÉS ---
        self.overtake_mode = False
        self.overtake_until = 0.0
        self.overtake_cooldown_until = 0.0
        self.OVERTAKE_DURATION = 0.0       
        self.OVERTAKE_COOLDOWN = 3.0       
        self.OVERTAKE_SPEED = 0.3         
        self.OVERTAKE_STEER = 1.5          

        # --- SEUILS DE DÉTECTION MODIFIÉS ---
        self.OVERTAKE_DIST_MIN = 0.2      
        self.OVERTAKE_DIST_MAX = 1.5       
        self.OVERTAKE_STAGNATION_TIME = 0.5      
        self.OVERTAKE_STAGNATION_DELTA = 0.25 

        # Historique d_front pour détecter la stagnation
        self.dfront_history = []
        self.DFRONT_HISTORY_SIZE = 15      
        self.stagnation_start = None

        self.last_log_time = 0.0
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)

    def _cb_ir_d(self, msg): self.ir_droit = msg.data if msg.data > 10.0 else 5.0
    def _cb_ir_g(self, msg): self.ir_gauche = msg.data if msg.data > 10.0 else 5.0

    def toggle_mode(self):
        new_mode = 0 if self._robot_mode == 1 else 1
        msg = Int32(); msg.data = new_mode
        self.mode_publisher.publish(msg); self._robot_mode = new_mode
        if new_mode == 0: self.stop_car()
        self.get_logger().info(f"MODE: {'ON' if new_mode==1 else 'OFF'}")

    def toggle_donut(self):
        if self._robot_mode != 1:
            self.get_logger().warn("⚠️ Active le robot d'abord (ESPACE) !")
            return
        self.donut_mode = not self.donut_mode
        if self.donut_mode:
            self.get_logger().warn(" DONUT MODE ON !")
        else:
            self.get_logger().warn(" DONUT MODE OFF — retour navigation")
            self.current_state = "AVANCE"
            self.prev_cte = 0.0

    def _cb_mode(self, msg):
        self._robot_mode = msg.data
        if self._robot_mode != 1:
            self.donut_mode = False
            self.overtake_mode = False
            self.stop_car()

    def stop_car(self):
        self.donut_mode = False
        self.overtake_mode = False
        msg = Twist(); msg.linear.x, msg.angular.z = 0.0, 0.0
        self.publisher.publish(msg)

    def check_stagnation(self, d_front, now):
        in_zone = self.OVERTAKE_DIST_MIN < d_front < self.OVERTAKE_DIST_MAX

        if not in_zone:
            self.stagnation_start = None
            self.dfront_history.clear()
            return False

        self.dfront_history.append(d_front)
        if len(self.dfront_history) > self.DFRONT_HISTORY_SIZE:
            self.dfront_history.pop(0)

        if len(self.dfront_history) < self.DFRONT_HISTORY_SIZE:
            return False

        variation = max(self.dfront_history) - min(self.dfront_history)

        if variation < self.OVERTAKE_STAGNATION_DELTA:
            if self.stagnation_start is None:
                self.stagnation_start = now
            elif now - self.stagnation_start >= self.OVERTAKE_STAGNATION_TIME:
                return True
        else:
            self.stagnation_start = None

        return False

    def listener_callback(self, msg):
        if self._robot_mode != 1: return
        ranges = msg.ranges
        if not ranges: return
        now = time.time()
        n = len(ranges)
        
        clean = [r if (0.07 < r < 4.5 and not math.isinf(r)) else 4.0 for r in ranges]
        
        raw_d = sum(clean[int(n*0.15):int(n*0.25)]) / (int(n*0.25) - int(n*0.15))
        raw_g = sum(clean[int(n*0.75):int(n*0.85)]) / (int(n*0.85) - int(n*0.75))
        d_front = min(clean[-35:] + clean[:35])

        cmd = Twist()

        # --- DONUT MODE : priorité absolue ---
        if self.donut_mode:
            cmd.linear.x = self.DONUT_SPEED
            cmd.angular.z = self.DONUT_STEER * self.DONUT_DIRECTION
            self.publisher.publish(cmd)
            if now - self.last_log_time > 0.3:
                self.get_logger().info(f"DONUT | Spd:{cmd.linear.x:.2f} Steer:{cmd.angular.z:.2f}")
                self.last_log_time = now
            return

        # --- NAVIGATION NORMALE ---
        if self.current_state != "TRANSITION":
            self.smooth_d = (self.alpha * raw_d) + ((1.0 - self.alpha) * self.smooth_d)
            self.smooth_g = (self.alpha * raw_g) + ((1.0 - self.alpha) * self.smooth_g)

        cte = (self.smooth_g - self.smooth_d)
        diff_cte = cte - self.prev_cte
        self.prev_cte = cte

        mode_label = "AUTO"

        if self.current_state == "AVANCE":
            # 🔥 AJOUT : Si on est en train de forcer l'évasion suite à une reprise, on applique l'ordre direct
            if now < self.evasion_until:
                cmd.linear.x = self.SPEED_SLOW
                cmd.angular.z = self.direction_evasion
                mode_label = "FORCED_EVASION"
                self.publisher.publish(cmd)
                return

            if d_front < 0.15:
                self.current_state = "STOP"; self.state_timer = now + 0.5
                self.reverse_steer = -1.0 if cte > 0 else 1.0
                
                # Détermination du côté libre (gauche vs droite) juste avant l'arrêt
                self.direction_evasion = 1.3 if raw_g > raw_d else -1.3
                
                self.overtake_mode = False
                self.stop_car(); return

            current_p = self.tau_p
            if d_front < 1.0: current_p = self.tau_p * 1.3
            steer = (current_p * cte) + (self.tau_d * diff_cte)

            # --- DÉTECTION OVERTAKE ---
            can_overtake = now > self.overtake_cooldown_until and not self.overtake_mode
            if can_overtake and self.check_stagnation(d_front, now):
                overtake_dir = 1.0 if raw_g > raw_d else -1.0
                side_label = "GAUCHE" if overtake_dir > 0 else "DROITE"
                self.get_logger().warn(f"🏎️ OVERTAKE {side_label} ! voiture détectée à {d_front:.2f}m")
                self.overtake_mode = True
                self.overtake_until = now + self.OVERTAKE_DURATION
                self.overtake_steer_dir = overtake_dir
                self.dfront_history.clear()
                self.stagnation_start = None

            # --- EXÉCUTION OVERTAKE ---
            if self.overtake_mode:
                if now < self.overtake_until:
                    cmd.linear.x = self.OVERTAKE_SPEED
                    cmd.angular.z = self.OVERTAKE_STEER * self.overtake_steer_dir
                    mode_label = f"OVERTAKE"
                else:
                    self.overtake_mode = False
                    self.overtake_cooldown_until = now + self.OVERTAKE_COOLDOWN
                    self.get_logger().info(" OVERTAKE TERMINÉ — retour navigation")

            # --- DRIFT ---
            if not self.overtake_mode:
                can_drift = now > self.drift_cooldown_until
                delta_cte = abs(diff_cte)  
                trigger_drift = (can_drift 
                    and (0.40 < d_front < 1.1)   
                    and (abs(cte) > 1.3) 
                    and (delta_cte > 0.15)        
                    and (now - self.last_drift_time > 2.5))
                
                if trigger_drift:
                    self.drift_until = now + self.drift_duration
                    self.last_drift_time = now

                if now < self.drift_until:
                    cmd.linear.x, limit_angle = 0.77, 1.9; mode_label = "DRIFT"
                else:
                    if d_front > 2.5: cmd.linear.x, limit_angle = self.SPEED_FAST, 1.1
                    elif d_front > 1.6: cmd.linear.x, limit_angle = self.SPEED_MED, 1.6
                    elif d_front > 0.5: cmd.linear.x, limit_angle = self.SPEED_SLOW, 2.0
                    else: cmd.linear.x, limit_angle = 0.07, 2.5
                
                cmd.angular.z = float(max(min(steer, limit_angle), -limit_angle))

        elif self.current_state == "STOP":
            cmd.linear.x, cmd.angular.z = 0.0, 0.0; mode_label = "STOP"
            if now > self.state_timer:
                self.current_state = "RECUL"; self.state_timer = now + 1.2

        elif self.current_state == "RECUL":
            cmd.linear.x = self.SPEED_REVERSE; mode_label = "RECUL"
            if self.ir_droit < 18.0:
                cmd.angular.z = self.STEER_AVOID_IR; mode_label = "RECUL_IR_D"
            elif self.ir_gauche < 18.0:
                cmd.angular.z = -self.STEER_AVOID_IR; mode_label = "RECUL_IR_G"
            else:
                cmd.angular.z = self.reverse_steer

            if now > self.state_timer and d_front > 0.4:
                self.current_state = "TRANSITION"
                self.state_timer = now + self.COOLDOWN_DURATION
                self.stop_car(); return

        elif self.current_state == "TRANSITION":
            cmd.linear.x, cmd.angular.z = 0.0, 0.0
            mode_label = "RESET_VISION"
            self.smooth_d, self.smooth_g, self.prev_cte = raw_d, raw_g, 0.0

            if now > self.state_timer and d_front > 0.5:
                self.drift_cooldown_until = now + 3.0
                self.get_logger().info(">>> REPARTI (DRIFT LOCK 3s) <<<")
                self.current_state = "AVANCE"
                
                # 🔥 CHANGEMENT : Au lieu d'envoyer un ordre unique, on arme le timer d'évasion
                self.evasion_until = now + self.EVASION_DURATION
                
                cmd.linear.x = self.SPEED_SLOW
                cmd.angular.z = self.direction_evasion
                self.publisher.publish(cmd)
                return

        self.publisher.publish(cmd)

        if now - self.last_log_time > 0.2:
            stag = f" STAG:{(now - self.stagnation_start):.1f}s" if self.stagnation_start else ""
            self.get_logger().info(
                f"[{mode_label}] F:{d_front:.2f}m | G:{raw_g:.2f} D:{raw_d:.2f} "
                f"| Steer:{cmd.angular.z:.2f}{stag}"
            )
            self.last_log_time = now


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
            if key == ' ':
                node.toggle_mode()
            elif key in ['w', 'W']:
                node.toggle_donut()
            elif key in ['q', 'Q', 's', 'S']:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_car()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
