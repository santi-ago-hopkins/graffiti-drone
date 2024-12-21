import rclpy
import csv
from rclpy.node import Node
from drone_msgs.msg import DebugMessage

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Create a subscriber to the DebugMessage topic
        self.sensor_subscriber = self.create_subscription(
            DebugMessage,
            '/csv',
            self.debug_callback,
            1
        )

        # Initialize the CSV file and write the header
        self.csv_file = open('debug_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time', 'Proportional', 'Derivative', 'Roll Error', 'Roll Derivative'])

        self.get_logger().info("CSV logging started. Data will be saved in 'debug_data.csv'.")

    def debug_callback(self, msg: DebugMessage):
        # Extract parameters from the DebugMessage
        prop_term = msg.proportional
        deriv_term = msg.derivative
        roll_error = msg.roll
        roll_deriv = msg.roll_deriv
        timestamp = msg.time

        # Write the parameters into the CSV file
        self.csv_writer.writerow([timestamp, prop_term, deriv_term, roll_error, roll_deriv])

        self.get_logger().info(
            f"Logged data: Time={timestamp}, Proportional={prop_term}, Derivative={deriv_term}, Roll Error={roll_error}, Roll Derivative={roll_deriv}"
        )

    def destroy_node(self):
        # Close the CSV file when the node is destroyed
        self.csv_file.close()
        self.get_logger().info("CSV file closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()

    try:
        rclpy.spin(pid_controller)
    except KeyboardInterrupt:
        pid_controller.get_logger().info("Node interrupted by user.")
    finally:
        pid_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

