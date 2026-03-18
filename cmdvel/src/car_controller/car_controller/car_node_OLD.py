import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time, math, sys, select, termios, tty

class NavTunnelNode(Node):
    def __init__(self):
        super().__init__('nav_tunnel_node')
        
        # --- RÉGLAGE DE LA DIRECTION ---
        self.INVERSION_DIRECTION = False  
        
        # --- CALIBRAGE PHYSIQUE ---
        # Ton réglage actuel (0.67 est assez élevé, signe d'un gros décalage physique)
        self.OFFSET_CENTRAGE = 0.67 
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        
        # Paramètres PID
        self.tau_p = 4.0   
        self.tau_d = 0.4   
        self.prev_cte = 0.0
        
        self.MIN_SPEED = 0.15 
        self.REVERSE_SPEED = -0.12 
        self.current_state = "AVANCE" 
        self.state_timer = 0.0
        self.last_log_time = 0.0

    def stop_car(self):
        self.publisher.publish(Twist())
        self.get_logger().error("🛑 STOP")

    def listener_callback(self, msg):
        ranges = msg.ranges
        if not ranges: return
        
        now = time.time()
        n = len(ranges)
        mid = n // 2
        exclusion = 0.12  

        # Nettoyage des données
        clean = [r if (exclusion < r < 4.0 and not math.isinf(r)) else 3.5 for r in ranges]

        # Vision devant
        zone_front = clean[-25:] + clean[:25]
        d_front = min(zone_front)

        # Vision Latérale
        avg_d = sum(clean[0:mid]) / mid
        avg_g = sum(clean[mid:n]) / mid

        cmd = Twist()

        # ==========================================================
        # LOGS ACCÉLÉRÉS : Changé de 0.5 à 0.1 pour plus de précision
        # ==========================================================
        if now - self.last_log_time > 0.1:
            self.get_logger().info(f"F:{d_front:.2f} | L:{avg_g:.2f} | R:{avg_d:.2f} | CTE:{((avg_g - avg_d) + self.OFFSET_CENTRAGE):.2f}")
            self.last_log_time = now

        if self.current_state == "AVANCE":
            if d_front < 0.20: 
                self.get_logger().warn(f"⚠️ MUR ({d_front:.2f}m)")
                self.current_state = "STOP"
                self.state_timer = now + 0.5
            else:
                # CALCUL DE L'ERREUR
                cte = (avg_g - avg_d) + self.OFFSET_CENTRAGE
                
                diff_cte = cte - self.prev_cte
                self.prev_cte = cte
                
                steer = (self.tau_p * cte) + (self.tau_d * diff_cte)
                
                cmd.linear.x = self.MIN_SPEED
                cmd.angular.z = float(max(min(steer, 0.8), -0.8))

        elif self.current_state == "STOP":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if now > self.state_timer:
                self.current_state = "RECUL"
                self.state_timer = now + 1.2

        elif self.current_state == "RECUL":
            cmd.linear.x = self.REVERSE_SPEED
            cmd.angular.z = 0.0
            if now > self.state_timer and d_front > 0.5:
                self.current_state = "AVANCE"

        if not math.isfinite(cmd.angular.z): cmd.angular.z = 0.0
        self.publisher.publish(cmd)

# --- GESTION CLAVIER ---
def is_key_pressed(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
    if rlist:
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return None

def main(args=None):
    rclpy.init(args=args)
    node = NavTunnelNode()
    settings = termios.tcgetattr(sys.stdin)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if is_key_pressed(settings) == ' ':
                node.stop_car()
                break
    except KeyboardInterrupt: pass
    finally:
        node.stop_car()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()