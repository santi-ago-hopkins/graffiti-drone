import rclpy
from rclpy.node import Node
from drone_msgs.msg import MotorCommand, SensorMessage, DistanceSensorArray

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Publishers:
        self.control_publisher = self.create_publisher(
            MotorCommand,
            '/cmd',
            1
        )
        
        self.send_final_command()

    def send_final_command(self):
        final_command = MotorCommand()
        final_command.motor1 = 0
        final_command.motor2 = 0
        final_command.motor3 = 0
        final_command.motor4 = 0
        self.control_publisher.publish(final_command)

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
