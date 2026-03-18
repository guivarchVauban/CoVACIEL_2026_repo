import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import time

class NavTunnelNode(Node):
    def __init__(self):
        super().__init__('nav_tunnel_node')
        self.get_logger().warning(">>> PILOTAGE AVEC RECUL : [ESPACE] POUR STOPPER <<<")

        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- PARAMÈTRES PID ---
        self.tau_p = 6.0    
        self.tau_d = 0.4    
        self.prev_cte = 0.0 

        # --- RÉGLAGES PHYSIQUES ---
        self.MAX_TURN = 0.80
        self.MIN_SPEED = 0.15
        self.REVERSE_SPEED = -0.15 # Vitesse négative pour reculer
        
        # --- ÉTAT DU ROBOT ---
        self.is_reversing = False
        self.reverse_until = 0.0

    def stop_car(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().error("🛑 ARRÊT COMPLET")

    def listener_callback(self, msg):
        ranges = msg.ranges
        if not ranges: return

        # --- 1. VISION (LiDAR) ---
        mid = len(ranges) // 2
        clean_ranges = [min(r, 3.5) if r > 0.1 else 3.5 for r in ranges]
        
        # Zones Latérales
        avg_droite = sum(clean_ranges[0 : mid]) / mid
        avg_gauche = sum(clean_ranges[mid : len(ranges)]) / mid
        
        # Zone Devant (30° de chaque côté)
        scan_devant = clean_ranges[mid-30 : mid+30]
        d_devant = min(scan_devant) if scan_devant else 3.5

        now = time.time()
        cmd = Twist()

        # --- 2. LOGIQUE DE RECUL (SÉCURITÉ) ---
        # Si on détecte un mur trop proche (< 25cm) et qu'on ne recule pas déjà
        if d_devant < 0.25 and not self.is_reversing:
            self.get_logger().info("🔄 OBSTACLE PROCHE ! RECUL EN COURS...")
            self.is_reversing = True
            self.reverse_until = now + 1.2  # Reculer pendant 1.2 secondes
        
        # Si on est en train de reculer
        if self.is_reversing:
            if now < self.reverse_until:
                cmd.linear.x = self.REVERSE_SPEED
                # On braque à l'inverse de là où il y a le plus d'espace pour se dégager
                cmd.angular.z = self.MAX_TURN if avg_droite > avg_gauche else -self.MAX_TURN
                self.publisher.publish(cmd)
                return # On saute le reste du code (PID) pendant le recul
            else:
                self.is_reversing = False
                self.get_logger().info("✅ DÉGAGEMENT TERMINÉ")

        # --- 3. CALCUL DU PID (MARCHE AVANT) ---
        cte = avg_gauche - avg_droite
        diff_cte = cte - self.prev_cte
        self.prev_cte = cte

        steer = (self.tau_p * cte) + (self.tau_d * diff_cte)

        # --- 4. COMMANDE MOTEUR ---
        cmd.linear.x = self.MIN_SPEED
        limit = 0.35 if d_devant > 2.0 else self.MAX_TURN
        cmd.angular.z = max(min(steer, limit), -limit)

        self.publisher.publish(cmd)

# --- GESTION DU CLAVIER ---
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
            rclpy.spin_once(node, timeout_sec=0.05)
            key = is_key_pressed(settings)
            if key == ' ':
                node.stop_car()
                break
    except KeyboardInterrupt:
        node.stop_car()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
