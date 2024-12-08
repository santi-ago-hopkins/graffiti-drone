import rclpy
import numpy as np
import re
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from drone_msgs.msg import MotorCommand, DistanceSensorArray, SensorMessage
import math

class Arduino(Node):
    def __init__(self):
        super().__init__('arduino_connection_node')

        self.control_subscriber = self.create_subscription(
            MotorCommand,
            '/cmd',
            self.cmd_callback,
            1
        )

        # IMU Data Publisher
        self.imu_publisher = self.create_publisher(
            SensorMessage,
            '/imu',
            1
        )
        self.distance_publisher = self.create_publisher(
            DistanceSensorArray, 
            '/distance_sensors',
            1
        )
        
        # Initialize controls
        self.acceleration = 0
        self.steering_angle = 0 
        self.serial_port = '/dev/serial/by-id/usb-Arduino_LLC_Arduino_MKRZero_C4260BF85030574B412E3120FF13161F-if00'  # Update this with serial port if changes
        self.baud_rate = 115200
        self.ser = serial.Serial(self.serial_port, self.baud_rate)

        # Create a timer that goes off every 0.001 seconds, and calls self.recieve_serial_values
        self.serial_timer = self.create_timer(0.001, self.recieve_serial_values)


    def cmd_callback(self, msg: MotorCommand):
        """ 
        Callback function that should recieve car commands from MPC.
        Sends the acceleration and steering values to the arduino through Serial.
        """
        motor1 = msg.motor1
        motor2 = msg.motor2
        motor3 = msg.motor3
        motor4 = msg.motor4

        print("Recieved Message")
        #Convert the steering value to what the motor has to rotate
        #:motor_message = f"{str(msg.motor1) + " " + str(msg.motor2) + " " + str(msg.motor3) + " " + str(msg.motor4)}" + '\n'
        motor_message = ""
        #self.ser.write(str(motor_str).encode('utf-8'))
        #self.ser.write(motor_str.encode())
        self.ser.write(bytearray(motor_message, 'ascii'))
        print("sent message")
        return 

    def recieve_serial_values(self):
        #Note: This probably isn't the most time efficient method of doing this, but I just wrote this in a day for push-testing
        if self.ser.in_waiting > 0: 
            data = self.ser.readline()


            # Regex for quaternions
            quaternion_pattern = r"qW: ([\d\.\-]+) qX: ([\d\.\-]+) qY: ([\d\.\-]+) qZ: ([\d\.\-]+)"
            gyroscope_pattern = r"Roll Rate:\s*([-+]?\d*\.\d+|\d+)\s*Pitch Rate:\s*([-+]?\d*\.\d+|\d+)\s*Yaw Rate:\s*([-+]?\d*\.\d+|\d+)"
            # Regex for depth
            depth_pattern = r"D=(\d+)mm"

            # Extract quaternions
            quaternion_match = re.search(quaternion_pattern, str(data))
            gyroscope_match = re.search(gyroscope_pattern, str(data))
            if quaternion_match and gyroscope_match:
                qW, qX, qY, qZ = map(float, quaternion_match.groups())
                print(f"qW: {qW}, qX: {qX}, qY: {qY}, qZ: {qZ}")
                roll, pitch, yaw = self.quat_to_euler((qW, qX, qY, qZ))
                imu_message = SensorMessage()
                imu_message.x = 0.0
                imu_message.y = 0.0
                imu_message.z = 0.0
                imu_message.roll = roll
                imu_message.pitch = pitch
                imu_message.yaw = yaw 

                roll_rate, pitch_rate, yaw_rate = map(float, gyroscope_match.groups())
                imu_message.roll_rate = roll_rate * np.pi / 180
                imu_message.pitch_rate = pitch_rate * np.pi / 180
                imu_message.yaw_rate = yaw_rate * np.pi /180
                self.imu_publisher.publish(imu_message)
            else:
                print("Quat not found.")
            # Extract depth
            depth_match = re.search(depth_pattern, str(data))
            if depth_match:
                depth = int(depth_match.group(1))  # Convert depth to integer
                distance_message = DistanceSensorArray()
                distance_message.x = 0.0
                distance_message.y = 0.0
                distance_message.z = depth / 1000
                self.distance_publisher.publish(distance_message)
                print(f"Depth: {depth} mm")
            else:
                print("Depth not found.")
            if gyroscope_match: 
                roll_rate = float(gyroscope_match.group(1))
                pitch_rate = float(gyroscope_match.group(2))
                yaw_rate = float(gyroscope_match.group(3))
                
    def quat_to_euler(self, quat):#: Quaternion) -> tuple[float, float, float]:
        x = quat[1]
        y = quat[2]
        z = quat[3]
        w = quat[0]

        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * x))
        pitch = math.asin(2.0 * (w * y - z * x))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        return roll, pitch, yaw
 
def main(args=None):
    rclpy.init(args=args)
    arduino_node = Arduino()
    rclpy.spin(arduino_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
