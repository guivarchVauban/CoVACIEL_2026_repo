import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # Type de message standard pour les Lidars
from std_msgs.msg import Int32         # Type de message pour envoyer l'angle à l'Arduino

class CarAutoPilot(Node):
    def __init__(self):
        # Initialise le noeud ROS 2 avec le nom 'car_autopilot_node'
        super().__init__('car_autopilot_node')
        
        # --- ABONNEMENT (SUBSCRIBER) ---
        # On s'abonne au topic '/scan'. Chaque fois que le Lidar envoie une donnée,
        # la fonction 'self.scan_callback' est appelée.
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10) # 10 est la taille de la file d'attente (QoS)
        
        # --- PUBLICATION (PUBLISHER) ---
        # On crée un objet pour envoyer des entiers sur le topic '/serv_cmd'.
        # C'est ce topic que ton noeud 'arduino_bridge' écoute.
        self.servo_pub = self.create_publisher(Int32, '/serv_cmd', 10)
        
        self.get_logger().info('✅ Pilote automatique avec Lidar activé !')

    def scan_callback(self, msg):
	
        # AJOUTE CETTE LIGNE ICI :
        self.get_logger().info("Données reçues !") 
        
        devant = msg.ranges[0:20] + msg.ranges[340:360]
        # ... le reste de ton code
        """
        Cette fonction s'exécute à chaque nouvelle lecture du Lidar (env. 7-10 fois par seconde).
        Le message 'msg' contient un tableau 'ranges' de 360 valeurs (distances en mètres).
        """
        
        # 1. ANALYSE DE LA ZONE DEVANT
        # Le Lidar scanne sur 360°. En général :
        # - 0° est juste devant.
        # - Les index 0 à 20 sont légèrement à gauche.
        # - Les index 340 à 360 sont légèrement à droite.
        # On combine ces deux tranches pour surveiller un cône de 40° face à la voiture.
        devant = msg.ranges[0:20] + msg.ranges[340:360]
        
        # 2. NETTOYAGE DES DONNÉES
        # On ignore les valeurs égales à 0 ou à l'infini (erreurs de lecture)
        # On ne garde que ce qui est supérieur à 10cm (0.1m) pour éviter de détecter le châssis.
        distances_valides = [d for d in devant if d > 0.1 and d < msg.range_max]
        
        if distances_valides:
            # On prend la distance la plus courte parmi les points valides
            distance_min = min(distances_valides)
            
            # 3. LOGIQUE DE DÉCISION
            if distance_min < 0.6:  # SEUIL DE DANGER : 60 cm
                # Si un obstacle est trop près, on envoie l'ordre de braquer
                self.get_logger().warn(f'⚠️ OBSTACLE ! Distance: {distance_min:.2f}m - Braquage !')
                
                # Création du message ROS et publication
                msg_angle = Int32()
                msg_angle.data = 135  # Angle maximum pour tourner (à ajuster selon ton servo)
                self.servo_pub.publish(msg_angle)
            
            else:
                # Si la route est libre, on remet les roues droites
                self.get_logger().info(f'🚀 Route libre ({distance_min:.2f}m) - Tout droit')
                
                msg_angle = Int32()
                msg_angle.data = 90   # 90° = Centre
                self.servo_pub.publish(msg_angle)

def main(args=None):
    # Initialisation de la communication ROS 2
    rclpy.init(args=args)
    
    # Création de notre intelligence artificielle
    node = CarAutoPilot()
    
    try:
        # Fait tourner le noeud en boucle jusqu'à un Ctrl+C
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Nettoyage propre au moment de quitter
        node.destroy_node()
        rclpy.shutdown()
