import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class NavTunnelNode(Node):
    def __init__(self):
        #definition des outils de com (sub/pub)
        #definition PID (kd et kp)
        super().__init__('nav_tunnel_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Gains PID stabilisés pour éviter l'effet serpent kp=0.6 kd=10
        self.Kp = 0.35  
        self.Kd = 15.0  
        self.last_error = 0.0

    def listener_callback(self, msg):
        #division des donnée du liadar en 3 secteur
        
        ranges = msg.ranges
        if not ranges:
            return

        num_points = len(ranges)
        mid = num_points // 2

        # 1. ANALYSE DES ZONES (Portée de 3.5m pour anticiper)
        # On regarde large pour ne jamais être "aveugle"
        #droite et gauche on cherche la distance la plus courte(min) par rapport au mur
        #Devant on surveille un petit cone central (mid-25 a mid+25)=angle de vision
        #standar ros l'indec 0 est souvent a droite 
        # if 0.10 < r < 3.5 c'est pour ignorer le bruit du lidar ou les info inutile
        scan_droite = [r for r in ranges[0 : num_points//3] if 0.15 < r < 3.5]
        scan_gauche = [r for r in ranges[2*num_points//3 : num_points] if 0.15 < r < 3.5]
        scan_devant = [r for r in ranges[mid-35 : mid+35] if 0.10 < r < 3.5]

        d_droite = min(scan_droite) if scan_droite else 2.5
        d_gauche = min(scan_gauche) if scan_gauche else 2.5
        d_devant = min(scan_devant) if scan_devant else 3.5
        #Twist() est comme un fichier de configuration ou un formulaire divisé en deux grandes colonnes vitesse et mouvement
        cmd = Twist()

        # 2. GESTION DE LA VITESSE : On ralentit si un mur approche en face
        if d_devant < 1.3:
            cmd.linear.x = 0.12  # Vitesse lente pour réussir le virage
        else:
            cmd.linear.x = 0.20  # Vitesse de croisière en ligne droite 0.2

        # 3. CALCUL DU PID POUR LE CENTRAGE
        error = d_gauche - d_droite
        
        # Deadzone minuscule pour stabiliser les roues en ligne droite =0.05
        if abs(error) < 0.12:
            error = 0.0
            
        derivative = error - self.last_error
        self.last_error = error
        w_final = (error * self.Kp) + (derivative * self.Kd)

        # 4. LOGIQUE DE DIRECTION
        if d_devant < 0.75:
            # Sécurité frontale : braquage prioritaire pour éviter l'impact
            cmd.angular.z = 0.14 if d_gauche > d_droite else -0.14
            self.get_logger().info(f"!!! ÉVITEMENT FRONTAL (Dist: {d_devant:.2f}) !!!")
        else:
            # Suivi de milieu fluide. Bridage à 0.18 pour éviter les warnings Webots
            cmd.angular.z = max(min(w_final, 0.10), -0.10)
            self.get_logger().info(f"D: {d_droite:.2f} | G: {d_gauche:.2f} | V: {cmd.linear.x:.2f}")

        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NavTunnelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()