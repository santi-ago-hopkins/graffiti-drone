import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from drone_msgs.msg import MotorCommand, SensorMessage

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Publishers:
        self.control_publisher = self.create_publisher(
            MotorCommand,
            '/cmd',
            1
        )
        
        self.sensor_subscriber = self.create_subscription(
            SensorMessage,
            '/sensors',
            self.sensor_callback,
            1
        )

        # Initial values for sensors
        self.init_roll = 0.0
        self.init_pitch = 0.0
        self.init_yaw = 0.0
        self.init_z = 0.0

        self.initializing = True
        self.init_samples = []
        self.init_start_time = self.get_clock().now()

        # Current states
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.goal_z = 10.0
        self.goal_yaw = 0.0
        self.goal_pitch = 0.0
        self.goal_roll = 0.0

        # Previous values for derivative calculations
        self.prev_time = self.get_clock().now()
        self.prev_z = None

        # Controller Parameters
        self.z_kp = 10.0
        self.z_kd = 10.0
        self.yaw_kp = 10.0
        self.yaw_kd = 10.0
        self.roll_kp = 10.0
        self.roll_kd = 10.0
        self.pitch_kp = 10.0
        self.pitch_kd = 10.0

        # Mixer Matrix 
        self.mixer_matrix = np.vstack(
            [[1, 1, 1, 1],
             [-1, 1, 1, -1],
             [1, -1, 1, -1], 
             [1, 1, -1, -1]])

    def sensor_callback(self, msg: SensorMessage):
        current_time = self.get_clock().now()

        # During initialization phase, collect sensor data
        if self.initializing:
            if (current_time - self.init_start_time).nanoseconds < 5e9:  # First 5 seconds
                self.init_samples.append((msg.roll, msg.pitch, msg.yaw, msg.z))
                return
            else:
                # Calculate averages and finalize initialization
                samples = np.array(self.init_samples)
                self.init_roll, self.init_pitch, self.init_yaw, self.init_z = np.mean(samples, axis=0)
                self.get_logger().info(f"Initialization complete: Roll={self.init_roll}, Pitch={self.init_pitch}, Yaw={self.init_yaw}, Z={self.init_z}")
                self.initializing = False
                return

        # Main control logic after initialization
        curr_time = current_time.nanoseconds * 1e-6  # milliseconds
        dt = curr_time - self.prev_time.nanoseconds * 1e-6
        
        # Get error terms
        z_error = self.goal_z - msg.z 
        z_dot = (self.z - self.prev_z) / dt if self.prev_z is not None and dt > 0 else 0.0
        thrust = self.z_kp * z_error + self.z_kd * z_dot

        roll_error = self.goal_roll - msg.roll
        yaw_error = self.goal_yaw - msg.yaw
        pitch_error = self.goal_pitch - msg.pitch

        # Calculate Inputs
        roll_input = self.roll_kp * roll_error + self.roll_kd * msg.roll_rate
        yaw_input = self.yaw_kp * yaw_error + self.yaw_kd * msg.yaw_rate
        pitch_input = self.pitch_kp * pitch_error + self.pitch_kd * msg.pitch_rate

        # Create Vector with Inputs 
        input_vector = np.array([thrust, roll_input, pitch_input, yaw_input]).T
        motor_input = self.mixer_matrix @ input_vector

        # Publish motor input
        control_message = MotorCommand()
        control_message.motor1 = motor_input[0]
        control_message.motor2 = motor_input[1]
        control_message.motor3 = motor_input[2]
        control_message.motor4 = motor_input[3]

        self.control_publisher.publish(control_message)


def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
