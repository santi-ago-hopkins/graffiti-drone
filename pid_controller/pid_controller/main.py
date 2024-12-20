import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray
from drone_msgs.msg import MotorCommand, SensorMessage, DistanceSensorArray, Debug
import pid_settings as settings

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Publishers:
        self.control_publisher = self.create_publisher(
            MotorCommand,
            '/cmd',
            1 
        )
        self.debug_publisher = self.create_publisher(
                Debug,
                '/csv',
                1
        )
 
        # Subscribers: 
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
        self.distance_subscriber = self.create_subscription(
            DistanceSensorArray,
            '/distance_sensors',
            self.distance_callback,
            1
        )
        self.initialized = False
        # State Vector is [x, y, z, roll, pitch, yaw]
        self.init_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.failsafe_active = False

        self.goal_state = np.array([0.0, 0.0, 0.5, 0.0, 0.0, 0.0])
        self.distance_sensor_state = np.array([0.0, 0.0, 0.0])
        self.attitude_state = np.array([0.0, 0.0, 0.0])
        # Derivative and Integral Arrays
        self.previous_error_vector = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.error_sum_vector = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Set previous time to use for derivative term
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        self.prev_time = seconds + (nanoseconds / 1e9)

        self.mixer_matrix = np.vstack(
            [[1, 1, 1, 1],
             [1, -1, -1, 1],
             [1, 1, -1, -1], 
             [1, -1, 1, -1]]).T


        self.controller_message_array = np.array([0, 0, 0, 0])
        
    def sensor_callback(self, msg: SensorMessage):
        # Get Initial Values of IMU to account for nonuniform mounting surface
        if not self.initialized:
            # NOTE: Doesn't matter if Z has read from the distance sensor yet
            self.init_state = np.array([0.0, 0.0, self.current_z, msg.roll, msg.pitch, msg.yaw])
            self.initialized = True
            self.get_logger().info("IMU initialized with reference values.")
            return

        # Don't send any more messages if failsafe was activated
        if self.failsafe_active == True: 
            return

        # Get current time for derivative term
        # NOTE: Nanoseconds roll over every second
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()

        curr_time = seconds + (nanoseconds / 1e9)
        dt = (curr_time - self.prev_time)
        self.prev_time = curr_time

        # Create Current State Vector
        attitude_state = np.array([msg.roll, msg.pitch, msg.yaw])
        self.attitude_state = attitude_state
        current_state = np.concatenate((self.distance_sensor_state, attitude_state))

        # Calculate Error Terms
        #NOTE: Gyroscope was being a little funky, so we just manually calculated derivatives
        error_vector = self.goal_state - current_state
        derivative_vector = (error_vector - self.previous_error_vector) / dt
        integral_vector = self.error_sum_vector + (error_vector * dt)
        
        # If we determine the drone roll or pitch is too far from equilibrium point, the system should be shut down
        if (np.abs(error_vector[3]) > 1.0 or np.abs(error_vector[5]) > 1.0): 
            self.get_logger().info("Drone drifted too far from equilibrium state, shutting down.")
            failsafe_message = MotorCommand()
            failsafe_message.motor1 = 0
            failsafe_message.motor2 = 0
            failsafe_message.motor3 = 0
            failsafe_message.motor4 = 0
            self.control_publisher.publish(failsafe_message)
            self.failsafe_active = True

        # Update Previous Error Vector
        self.previous_error_vector = error_vector


        # Dynamically select pitch_kd and pitch_kp
        pitch_kd = self.sigmoid(derivative_vector[4], settings.PITCH_NEG_KD, settings.PITCH_POS_KD, settings.PITCH_KP_SIGMOID_K)
        pitch_kp = self.sigmoid(error_vector[4], settings.PITCH_NEG_KP, settings.PITCH_POS_KP, settings.PITCH_KD_SIGMOID_K)

        Kp_vector = [pitch_kp if i == 4 else val for i, val in enumerate(settings.KP_VECTOR)]
        Kd_vector = [pitch_kd if i == 4 else val for i, val in enumerate(settings.KD_VECTOR)]
        Ki_vector = settings.KI_VECTOR

        # Calculate Inputs
        input_vector = error_vector * Kp_vector + derivative_vector * Kd_vector + integral_vector * Ki_vector

        # For now, just send thrust, roll, pitch, yaw
        mixer_input = input_vector[2:]
        mixer_input[0] += settings.HOVER_THROTTLE

        # Generate Motor Inputs
        motor_input = self.mixer_matrix @ mixer_input.T

        # If one motor command is negative, add the absolute value to the opposing motor
        control_message = MotorCommand()
        if motor_input[0] < 0.0: 
            motor_input[1] += np.abs(motor_input[0])
        if motor_input[1] < 0.0: 
            motor_input[0] += np.abs(motor_input[1])
        if motor_input[2] < 0.0: 
            motor_input[3] += np.abs(motor_input[2])
        if motor_input[3] < 0.0: 
            motor_input[2] += np.abs(motor_input[3])

        # Make 10 the minimum value, because motor takes time to start spinning from standstill
        control_message.motor1 = max(int(motor_input[0]), 10)
        control_message.motor2 = max(int(motor_input[1]), 10)
        control_message.motor3 = max(int(motor_input[2]), 10)
        control_message.motor4 = max(int(motor_input[3]), 10)
        
        self.control_publisher.publish(control_message)


        # Debug Portion # 
        # debug_message = Debug()
        # debug_message.proportional = float(pitch_error * pitch_kp)
        # debug_message.derivative = float(pitch_deriv * pitch_kd)
        # debug_message.integral = float(self.pitch_error_sum * self.pitch_ki)
        # debug_message.roll = float(msg.pitch)
        # debug_message.roll_error = float(pitch_error)
        # debug_message.roll_deriv = float(pitch_deriv * pitch_kd)
        # debug_message.time = curr_time * 1e-9

        # self.debug_publisher.publish(debug_message)

    #give us rotation matrix, given yaw pitch roll, where theta is 1x3 array
    def eulerAnglesToRotationMatrix(self, theta) :
    
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

    def distance_callback(self, msg):
        #set thetas
        thetas = self.attitude_state

        #get rotation matrix
        R = self.eulerAnglesToRotationMatrix(thetas)

        #find z_dist 
        dist_vec = np.dot(R, [msg.x, msg.y, msg.z])
        self.distance_sensor_state = dist_vec

    def path_callback(self, msg: PoseArray):
        coordinate_transform = np.vstack([0, 0, 1],
                                         [1, 0, 0],
                                         [0, 1, 0])
        msg_vector = [msg.poses.x, msg.poses.y, msg.poses.z] 
        goal_vector = coordinate_transform @ msg_vector
        self.goal_z = goal_vector[2]
        self.goal_y = goal_vector[1]

    def sigmoid(self, x, lower_bound, upper_bound, k):
        """
        Standard Helper Sigmoid Function
        """
        return lower_bound + ((upper_bound - lower_bound) / (1 + (np.exp(k * x))))

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()