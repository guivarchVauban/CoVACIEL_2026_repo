#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import struct
import math

ICM20600_ADDR = 0x69
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

ACCEL_SCALE = 16384.0   # LSB/g  — plage ±2g par défaut
GYRO_SCALE  = 131.0     # LSB/°/s — plage ±250°/s par défaut
G           = 9.80665

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.bus = smbus2.SMBus(1)           # bus 1 = GPIO 2/3 du Pi
        self.bus.write_byte_data(ICM20600_ADDR, PWR_MGMT_1, 0x00)  # wake up
        self.create_timer(0.01, self.read_imu)   # 100 Hz
        self.get_logger().info('node_imu démarré — ICM-20600 @ 0x68')

    def _read6(self, reg):
        data = self.bus.read_i2c_block_data(ICM20600_ADDR, reg, 6)
        return struct.unpack('>3h', bytes(data))   # 3 × int16 big-endian

    def read_imu(self):
        try:
            ax, ay, az = self._read6(ACCEL_XOUT_H)
            gx, gy, gz = self._read6(GYRO_XOUT_H)

            msg = Imu()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # Accéléromètre → m/s²
            msg.linear_acceleration.x = ax / ACCEL_SCALE * G
            msg.linear_acceleration.y = ay / ACCEL_SCALE * G
            msg.linear_acceleration.z = az / ACCEL_SCALE * G

            # Gyroscope → rad/s
            msg.angular_velocity.x = gx / GYRO_SCALE * math.pi / 180.0
            msg.angular_velocity.y = gy / GYRO_SCALE * math.pi / 180.0
            msg.angular_velocity.z = gz / GYRO_SCALE * math.pi / 180.0

            # Pas de fusion ici → orientation inconnue (convention ROS)
            msg.orientation_covariance[0] = -1.0

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Erreur lecture IMU : {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
