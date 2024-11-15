import rclpy
import numpy as np
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Publishers:
        self.control_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/cmd',
            1
        )
        
        # Subscribers:
        self.distance_sensor_subscriber = self.create_subscriber(
            MessageType,
            '/sensor_array',
            self.state_callback,
            1
        )
        self.imu_subscriber = self.create_subscriber(
            IMU,
            '/Imu',
            self.imu_callback,
            1
        )

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

