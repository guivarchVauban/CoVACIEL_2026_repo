import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import tf2_ros

class LaserScanToPointCloud(Node):
    def __init__(self):
        super().__init__('laser_scan_to_pc')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(PointCloud2, '/scan_pc', 10)
        self.projector = LaserProjection()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def scan_callback(self, msg):
        try:
            cloud = self.projector.projectLaser(msg)
            self.publisher.publish(cloud)
        except Exception as e:
            self.get_logger().warn(f"Projection failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()