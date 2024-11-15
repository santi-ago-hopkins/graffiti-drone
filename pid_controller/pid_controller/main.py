import rclpy
import numpy as np
from rclpy.node import Node
import rospy
import serial
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
        
        self.sensor_subscriber = self.create_subscriber(
            SensorMessage,
            '/sensors',
            self.sensor_callback,
            1
        )

    
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
        
        # Set previous time to use for derivative term
        self.prev_time = Node.get_clock().now() * 10**6
        self.prev_z = None

        # Controller Parameters
        # ------------------------------------------    
        z_kp = 10.0
        z_kd = 10.0

        yaw_kp = 10.0
        yaw_kd = 10.0

        roll_kp = 10.0
        roll_kd = 10.0 

        pitch_kp = 10.0
        pitch_kd = 10.0

        # Mixer Matrix 
        # ----------------------------------------------
        # [1, -1,  1,  1  ]
        # [1,  1, -1,  1  ]
        # [1,  1,  1, -1  ]
        # [1, -1, -1, -1  ]
        mixer_matrix = np.vstack(
            [[1, 1, 1, 1],
            [-1, 1, 1, -1],
            [1, -1, 1, -1], 
            [1, 1, -1, -1]])  
    
    def state_callback(self, msg: SensorMessage):
        # Get current time for derivative term
        curr_time = Node.get_clock().now()
        dt = (curr_time - self.prev_time).nanoseconds * 1e-6 # get delta time in milliseconds
        # Get error terms of all the directions
        z_error = self.goal_z - SensorMessage.z 
        if dt > 0:
            z_dot = (self.z - self.prev_z) / dt
            thrust = self.z_kp * z_error + self.roll_kd * z_dot
        # x_error
        # y_error
        roll_error = self.goal_roll - SensorMessage.roll
        yaw_error = self.goal_yaw - SensorMessage.yaw
        pitch_error = self.goal_pitch - SensorMessage.pitch
        
        # Calculate Inputs for each term
        roll_input = self.roll_kp * roll_error + self.roll_kd * SensorMessage.roll_rate
        yaw_input = self.yaw_kp * yaw_error + self.yaw_kd * SensorMessage.yaw_rate
        pitch_input = self.pitch_kp * yaw_error + self.pitch_kd * SensorMessage.pitch_rate
        
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

