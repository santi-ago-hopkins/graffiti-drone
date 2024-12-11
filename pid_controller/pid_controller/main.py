import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs import PoseArray
from drone_msgs.msg import MotorCommand, SensorMessage, DistanceSensorArray
import matplotlib.pyplot as plt 
import math

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

        self.planning_subscriber = self.create_subsciption(
            Pose,
            '/path',
            self.path_callback,
            1
        )

        self.planning_publisher = self.create_publisher(
            Int32, 
            '/visit',
            1
        )


        self.initialized = False
        self.init_roll = 0.0
        self.init_pitch = 0.0
        self.init_yaw = 0.0
        self.init_x = 0.0
        self.init_y = 0.0
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

        self.hover_throttle = 1400.0

        # Set previous time to use for derivative term
        self.prev_time = self.get_clock().now().nanoseconds * 10**6
        self.prev_z = None

        # Controller Parameters
        # ------------------------------------------    
        self.z_kp = 100.0
        self.z_kd = 10.0

        self.yaw_kp = 10.0
        self.yaw_kd = 1.0

        self.roll_kp = 10.0
        self.roll_kd = 1.0

        self.pitch_kp = 10.0
        self.pitch_kd = 1.0

        self.current_z = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        # Mixer Matrix 
        # ----------------------------------------------
        # [1, -1,  1,  1  ]
        # [1,  1, -1,  1  ]
        # [1,  1,  1, -1  ]
        # [1, -1, -1, -1  ]
        self.mixer_matrix = np.vstack(
            [[1, 1, 1, 1],
             [-1, 1, 1, -1],
             [1, -1, 1, -1], 
             [1, 1, -1, -1]])


        #self.visualize_drone_forces_realtime()
        self.controller_message_array = np.array([0, 0, 0, 0])

        #planning 
        self.error_threshold = 1.0 #change me to set error threshold to move onto next point
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.ind_count = 0

    def sensor_callback(self, msg: SensorMessage):

        # # During initialization phase, collect sensor data
        # if self.initializing:
        #     if (current_time - self.init_start_time).nanoseconds < 5e9:  # First 5 seconds
        #         self.init_samples.append((msg.roll, msg.pitch, msg.yaw, msg.z))
        #         return
        #     else:
        #         # Calculate averages and finalize initialization
        #         samples = np.array(self.init_samples)
        #         self.init_roll, self.init_pitch, self.init_yaw, self.init_z = np.mean(samples, axis=0)
        #         self.get_logger().info(f"Initialization complete: Roll={self.init_roll}, Pitch={self.init_pitch}, Yaw={self.init_yaw}, Z={self.init_z}")
        #         self.initializing = False
        #         return

        if not self.initialized:
            self.init_roll = msg.roll
            self.init_pitch = msg.pitch
            self.init_yaw = msg.yaw
            self.init_z = self.current_z
            self.init_y = self.current_y
            self.init_x = self.current_x
            self.initialized = True
            self.get_logger().info("IMU initialized with zero reference values.")
            return
        
        #get yaw, roll, pitch
        self.yaw = msg.yaw
        self.roll = msg.roll
        self.pitch = msg.pitch

        #get x, y, z
        self.x += msg.x
        self.y += msg.y
        self.z += msg.z

        # Get current time for derivative term
        curr_time = self.get_clock().now().nanoseconds
        dt = (curr_time - self.prev_time) * 1e-6 # get delta time in milliseconds
        
        # Get error terms
        # z_error = self.goal_z - self.current_z 
        z_error = self.goal_z - self.z
        z_dot = (self.z - self.prev_z) / dt if self.prev_z is not None and dt > 0 else 0.0
        thrust = self.z_kp * z_error + self.z_kd * z_dot

        #add in x, y error (this leads to no input, just to track whether or not we've reached the point)
        x_error = self.goal_x - self.x
        y_error = self.goal_y - self.y

        #collect cartesian errors
        cart_errors = [x_error, y_error, z_error]
        
        #send current point if are errors are small enough!
        if all(error < self.error_threshold for error in cart_errors):
            print('moving onto next point!')
            self.ind_count += 1
            self.planning_publisher.publish(self.ind_count)


        #roll, pitch, yaw errors
        roll_error = self.goal_roll - msg.roll
        yaw_error = self.goal_yaw - msg.yaw
        pitch_error = self.goal_pitch - msg.pitch

        # Calculate Inputs
        roll_input = self.roll_kp * roll_error + self.roll_kd * msg.roll_rate
        yaw_input = self.yaw_kp * yaw_error + self.yaw_kd * msg.yaw_rate
        pitch_input = self.pitch_kp * pitch_error + self.pitch_kd * msg.pitch_rate

        # Create Vector with Inputs 
        input_vector = np.array([self.hover_throttle + thrust, roll_input, pitch_input, yaw_input]).T
        motor_input = self.mixer_matrix @ input_vector

        # Publish motor input
        control_message = MotorCommand()
        control_message.motor1 = int(motor_input[0])
        control_message.motor2 = int(motor_input[1])
        control_message.motor3 = int(motor_input[2])
        control_message.motor4 = int(motor_input[3])

        self.control_publisher.publish(control_message)

        # Visualizer Section # 
        #######################

    #give us rotation matrix, given yaw pitch roll, where theta is 1x3 array
    def eulerAnglesToRotationMatrix(theta) :
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
        thetas = [self.roll, self.pitch, self.yaw]

        #get rotation matrix
        R = eulerAnglesToRotationMatrix(thetas)

        #find z_dist 
        dist_vec = np.dot(R, [0, 0, msg.z])
        self.current_z = dist_vec[2]

    def path_callback(self, msg: Pose):
        coordinate_transform = np.vstack([0, 0, 1],
                                         [1, 0, 0],
                                         [0, 1, 0])
        msg_vector = [msg.poses.x, msg.poses.y, msg.poses.z] 
        goal_vector = coordinate_transform@msg_vector
        self.goal_z = goal_vector[2]
        self.goal_y = goal_vector[1]
        

    # def visualize_drone_forces_realtime(self):
    #     """
    #     Visualize the forces acting on a drone's four motors in real time.
    #     """
    #     # Drone layout
    #     positions = {
    #         "front-left": (0.5, 1),
    #         "front-right": (1.5, 1),
    #         "back-right": (1.5, 0),
    #         "back-left": (0.5, 0)
    #     }

    #     # Setup the plot
    #     fig, ax = plt.subplots(figsize=(6, 6))
    #     ax.set_xlim(-0.5, 2)
    #     ax.set_ylim(-0.5, 2)

    #     # Draw drone body
    #     drone_body = plt.Circle((1, 0.5), 0.2, color='black', alpha=0.6)
    #     ax.add_patch(drone_body)

    #     arrows = {}
    #     labels = {}

    #     # Initialize arrows and labels
    #     for idx, (motor, position) in enumerate(positions.items()):
    #         arrow = ax.arrow(
    #             position[0], position[1], 0, 0, head_width=0.1, head_length=0.1, fc='gray', ec='gray'
    #         )
    #         label = ax.text(position[0] - 0.2, position[1] + 0.1, "0.00", fontsize=10)
    #         arrows[motor] = arrow
    #         labels[motor] = label

    #     ax.set_title("Drone Motor Force Visualization (Real Time)")
    #     ax.set_xlabel("X-axis")
    #     ax.set_ylabel("Y-axis")
    #     ax.grid(True)
    #     plt.gca().set_aspect('equal', adjustable='box')

    #     plt.ion()  # Enable interactive mode
    #     plt.show()

    #     while True:
    #         forces = self.controller_message_array

    #         max_force = max(forces) if max(forces) != 0 else 1
    #         normalized_forces = [f / max_force for f in forces]

    #         for idx, (motor, position) in enumerate(positions.items()):
    #             force = normalized_forces[idx]

    #             # Update arrow
    #             arrows[motor].remove()  # Remove the old arrow
    #             arrows[motor] = ax.arrow(
    #                 position[0], position[1], 0, 0.5 * force,
    #                 head_width=0.1, head_length=0.1, fc='blue', ec='blue'
    #             )

    #             # Update label
    #             labels[motor].set_text(f"{forces[idx]:.2f}")

    # visualize_drone_forces_realtime()

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
