import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter


class NavTunnelNode(Node):

    def __init__(self):
        super().__init__('nav_tunnel_node')

        # Utiliser le temps simulation (Webots)
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.get_logger().warning("🧪 MODE CRASH TEST : RECUL AUTOMATIQUE")

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # ===== PARAMÈTRES RECOVERY =====
        self.is_reversing = False
        self.stuck_start_time = None
        self.reverse_start_time = None

        self.TIME_TO_STUCK = 1.5
        self.REVERSE_DURATION = 1.5
        self.STUCK_THRESHOLD = 0.45

        # ===== PARAMÈTRES MOTEUR =====
        self.MIN_SPEED = 0.15
        self.MAX_TURN = 0.0

    def listener_callback(self, msg):

        now_time = self.get_clock().now()
        now = now_time.nanoseconds / 1e9

        ranges = msg.ranges
        if not ranges:
            return

        mid = len(ranges) // 2

        # ======================================================
        # NETTOYAGE LIDAR
        # ======================================================
        clean_ranges = []

        for r in ranges:
            if r == float('inf') or r > 3.5:
                clean_ranges.append(3.5)
            elif r < 0.05:
                clean_ranges.append(3.5)
            else:
                clean_ranges.append(r)

        # ======================================================
        # VISION DEVANT (CÔNE LARGE)
        # ======================================================
        front_arc = clean_ranges[mid-120:mid+120]

        d_devant = min(front_arc) if front_arc else 3.5
        d_min_global = min(clean_ranges)

        self.get_logger().info(
            f"DEVANT: {d_devant:.2f} | MIN SCAN: {d_min_global:.2f}"
        )

        # ======================================================
        # MODE RECUL
        # ======================================================
        if self.is_reversing:

            elapsed_rev = now - self.reverse_start_time

            if elapsed_rev < self.REVERSE_DURATION:

                cmd = Twist()
                cmd.linear.x = -0.20
                cmd.angular.z = 0.0

                self.publisher.publish(cmd)
                self.get_logger().warning(f"↩️ RECUL {elapsed_rev:.2f}s")

                return

            else:

                self.is_reversing = False
                self.stuck_start_time = None

                self.get_logger().info("✅ FIN DU RECUL")

        # ======================================================
        # DÉTECTION BLOCAGE
        # ======================================================
        if d_devant < self.STUCK_THRESHOLD:

            if self.stuck_start_time is None:

                self.stuck_start_time = now
                self.get_logger().warning("⏱️ CHRONO BLOCAGE DÉMARRÉ")

            elapsed = now - self.stuck_start_time

            self.get_logger().info(
                f"⏳ Bloqué: {elapsed:.2f}s / {self.TIME_TO_STUCK}s"
            )

            if elapsed > self.TIME_TO_STUCK:

                self.get_logger().error("🚨 BLOCAGE CONFIRMÉ → RECUL")

                self.is_reversing = True
                self.reverse_start_time = now

                cmd = Twist()
                cmd.linear.x = -0.20
                cmd.angular.z = 0.0

                self.publisher.publish(cmd)

                return

        else:

            if d_devant > 1.0:

                if self.stuck_start_time is not None:
                    self.get_logger().info("🍀 VOIE DÉGAGÉE → RESET CHRONO")

                self.stuck_start_time = None

        # ======================================================
        # AVANCER NORMAL
        # ======================================================
        cmd = Twist()

        cmd.linear.x = self.MIN_SPEED
        cmd.angular.z = self.MAX_TURN

        self.publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = NavTunnelNode()

    try:

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:

        node.get_logger().warning("Ctrl+C détecté")

    finally:

        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        node.publisher.publish(stop)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()