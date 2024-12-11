import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray
from drone_msgs.msg import MotorCommand, SensorMessage, DistanceSensorArray
import threading
import math

def eulerAnglesToRotationMatrix(theta):
    
        R_x = np.array([[1,         0,                  0                   ],
                        [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                        [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                        ])
    
        R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                        [0,                     1,      0                   ],
                        [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                        ])
    
        R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                        [math.sin(theta[2]),    math.cos(theta[2]),     0],
                        [0,                     0,                      1]
                        ])
    
        R = np.dot(R_z, np.dot( R_y, R_x ))
    
        return R


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
            '/imu',
            self.sensor_callback,
            1
        )
        self.planning_subscriber = self.create_subscription(
            PoseArray,
            '/path',
            self.path_callback,
            1
        )
        self.initialized = False
        self.init_roll = 0.0
        self.init_pitch = 0.0
        self.init_yaw = 0.0
        self.init_z = 0.0
        # Current states
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.goal_z = 0.5
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.goal_pitch = 0.0
        self.goal_roll = 0.0
        self.hover_throttle = 1300.0
        # Controller Parameters
        # ------------------------------------------
        self.z_kp = 0.0
        self.z_kd = 0.0
        self.yaw_kp = 0.0
        self.yaw_kd = 0.0
        self.roll_kp = 135
        self.roll_kd = 0.0
        self.pitch_kp = 135
        self.pitch_kd = 0.0
        self.current_z = 0.0
        # --------------------------------------------
        # Mixer Matrix
        self.mixer_matrix = np.vstack(
            [[1, -1, 1, 1],
             [1, 1, -1, 1],
             [1, 1, 1, -1],
             [1, -1, -1, -1]])
        self.controller_message_array = np.array([0, 0, 0, 0])

    def sensor_callback(self, msg: SensorMessage):
        if not self.initialized:
            self.init_roll = msg.roll
            self.init_pitch = msg.pitch
            self.init_yaw = msg.yaw
            self.init_z = self.current_z
            self.initialized = True
            self.get_logger().info("IMU initialized with zero reference values.")
            return
        self.yaw = msg.yaw
        self.roll = msg.roll
        self.pitch = msg.pitch
        curr_time = self.get_clock().now().nanoseconds * 1e-6
        #dt = (curr_time - self.prev_time) if curr_time > self.prev_time else 1.0
        # Get error terms
        z_error = self.goal_z - self.current_z
        #z_dot = (self.z - self.prev_z) / dt if self.prev_z is not None and dt > 0 else 0.0
        thrust = self.z_kp * z_error + self.z_kd# * z_dot
        roll_error = self.goal_roll - msg.roll
        yaw_error = self.goal_yaw - msg.yaw
        pitch_error = self.goal_pitch - msg.pitch
        print("ERRORS: ")
        print("Roll: ", roll_error)
        print("PITCH: ", pitch_error)
        print("Yaw: ", yaw_error)
        # Calculate Inputs
        roll_input = self.roll_kp * roll_error + self.roll_kd * msg.roll_rate
        yaw_input = 0#self.yaw_kp * yaw_error + self.yaw_kd * msg.yaw_rate
        pitch_input = self.pitch_kp * pitch_error + self.pitch_kd * msg.pitch_rate
        #print("INPUTS: ")
        print("ROLL INPUT: ", roll_input)
        print("PITCH INPUT: ", pitch_input)
        # Create Vector with Inputs
        input_vector = np.array([self.hover_throttle + thrust, roll_input, pitch_input, yaw_input]).T
        motor_input = self.mixer_matrix @ input_vector
        print(motor_input)
        control_message = MotorCommand()
        control_message.motor1 = min(int(motor_input[2]), 1600)
        control_message.motor2 = min(int(motor_input[3]), 1600)
        control_message.motor3 = min(int(motor_input[0]), 1600)
        control_message.motor4 = min(int(motor_input[1]), 1600)
        self.control_publisher.publish(control_message)

    def distance_callback(self, msg):
        #set thetas
        thetas = [self.roll, self.pitch, self.yaw]
        #get rotation matrix
        R = eulerAnglesToRotationMatrix(thetas)
        #find z_dist
        dist_vec = np.dot(R, [0, 0, msg.z])
        self.current_z = dist_vec[2]

    def path_callback(self, msg: PoseArray):
        coordinate_transform = np.vstack([0, 0, 1],
                                         [1, 0, 0],
                                         [0, 1, 0])
        msg_vector = [msg.poses.x, msg.poses.y, msg.poses.z]
        goal_vector = coordinate_transform@msg_vector
        self.goal_z = goal_vector[2]
        self.goal_y = goal_vector[1]

    def update_kp_live(self):
        while True:
            try:
                new_kp = int(input("Enter new kp value: "))
                self.roll_kp = new_kp
                self.pitch_kp = new_kp
                self.get_logger().info(f"Updated kp to {self.roll_kp}")
            except ValueError:
                self.get_logger().error("Invalid input. Please enter a numeric value.")

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    rclpy.spin(controller)
    rclpy.shutdown()
if __name__ == '__main__':
    main()