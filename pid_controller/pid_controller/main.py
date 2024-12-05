import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from drone_msgs.msg import MotorCommand, SensorMessage, DistanceSensorArray
import matplotlib.pyplot as plt 
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
        self.distance_sensor_subscriber = self.create_subscription(
            DistanceSensorArray,
            '/distance_sensors',
            self.distance_callback,
            1
        )
        # Initial values for sensors
        self.init_roll = 0.0
        self.init_pitch = 0.0
        self.init_yaw = 0.0
        self.init_z = 0.0

        self.initializing = True
        self.init_samples = np.array([])
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

        self.current_z = 0.0
        # Mixer Matrix 
        self.mixer_matrix = np.vstack(
            [[1, 1, 1, 1],
             [-1, 1, 1, -1],
             [1, -1, 1, -1], 
             [1, 1, -1, -1]])


        #self.visualize_drone_forces_realtime()
        self.controller_message_array = np.array([0, 0, 0, 0])
    def sensor_callback(self, msg: SensorMessage):
        current_time = self.get_clock().now()

        # # During initialization phase, collect sensor data
        # if self.initializing:
        #     if (current_time - self.init_start_time).nanoseconds < 5e9:  # First 5 seconds
        #         np.append(self.init_samples, (msg.roll, msg.pitch, msg.yaw, msg.z))
        #         return
        #     else:
        #         # Calculate averages and finalize initialization
        #         samples = np.array(self.init_samples)
        #         self.init_roll, self.init_pitch, self.init_yaw, self.init_z = np.mean(samples, axis=0)
        #         self.get_logger().info(f"Initialization complete: Roll={self.init_roll}, Pitch={self.init_pitch}, Yaw={self.init_yaw}, Z={self.init_z}")
        #         self.initializing = False
        #         return

        # Main control logic after initialization
        curr_time = current_time.nanoseconds * 1e-6  # milliseconds
        dt = curr_time - self.prev_time.nanoseconds * 1e-6
        
        # Get error terms
        z_error = self.goal_z - self.current_z
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
        control_message.motor1 = int(motor_input[0])
        control_message.motor2 = int(motor_input[1])
        control_message.motor3 = int(motor_input[2])
        control_message.motor4 = int(motor_input[3])

        self.controller_message_array = motor_input
        self.control_publisher.publish(control_message)

        # Visualizer Section # 
        #######################

    def distance_callback(self, msg):
        self.current_z = msg.z
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
