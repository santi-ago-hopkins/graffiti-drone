import rclpy
import re
from rclpy.node import Node
import serial
from drone_msgs.msg import MotorCommand, DistanceSensorArray, SensorMessage
import math
import numpy as np

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
        
        self.serial_port = '/dev/serial/by-id/usb-Arduino_LLC_Arduino_MKRZero_C4260BF85030574B412E3120FF13161F-if00'
        self.baud_rate = 115200
        self.ser = serial.Serial(self.serial_port, self.baud_rate)

        # Create a timer that goes off every 0.001 seconds, and calls self.recieve_serial_values
        # Created this way because cannot create a while True loop before Node initialization finishes
        self.serial_timer = self.create_timer(0.001, self.recieve_serial_values)

    def cmd_callback(self, msg: MotorCommand):
        motor1 = msg.motor1
        motor2 = msg.motor2
        motor3 = msg.motor3
        motor4 = msg.motor4
        # Write Motor Message that Arduino is expecting
        motor_message = f"{str(msg.motor1) + "/" + str(msg.motor2) + "/" + str(msg.motor3) + "/" + str(msg.motor4) + "/"}" + '\n'
        #self.ser.write(str(motor_str).encode('utf-8'))
        #self.ser.write(motor_str.encode())
        self.ser.write(bytearray(motor_message, 'ascii'))
        return 

    def recieve_serial_values(self):
        if self.ser.in_waiting > 0: 
            data = self.ser.readline()

            # Regex for different message patterns
            quaternion_pattern = r"qW: ([\d\.\-]+) qX: ([\d\.\-]+) qY: ([\d\.\-]+) qZ: ([\d\.\-]+)"
            gyroscope_pattern = r"Roll Rate:\s*([-+]?\d*\.\d+|\d+)\s*Pitch Rate:\s*([-+]?\d*\.\d+|\d+)\s*Yaw Rate:\s*([-+]?\d*\.\d+|\d+)"
            depth_pattern = r"D=(\d+)mm"

            # Extract quaternions
            quaternion_match = re.search(quaternion_pattern, str(data))
            gyroscope_match = re.search(gyroscope_pattern, str(data))
            if quaternion_match and gyroscope_match:
                qW, qX, qY, qZ = map(float, quaternion_match.groups())
                #print(f"qW: {qW}, qX: {qX}, qY: {qY}, qZ: {qZ}")
                try: 
                    roll, pitch, yaw = self.quat_to_euler((qW, qX, qY, qZ))
                except:
                    print("Quaternion Conversion Failed")
                    return
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
                
            # Extract depth
            depth_match = re.search(depth_pattern, str(data))
            if depth_match:
                depth = int(depth_match.group(1))
                distance_message = DistanceSensorArray()
                distance_message.x = 0.0
                distance_message.y = 0.0
                distance_message.z = depth / 1000
                self.distance_publisher.publish(distance_message)
                #print(f"Depth: {depth} mm")
            if gyroscope_match: 
                roll_rate = float(gyroscope_match.group(1))
                pitch_rate = float(gyroscope_match.group(2))
                yaw_rate = float(gyroscope_match.group(3))
                
    def quat_to_euler(self, quat):
        """
        Helper function that converts quaternions to roll pitch yaw
        Pulled from Stack Overflow
        https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
        """
        x = quat[1]
        y = quat[2]
        z = quat[3]
        w = quat[0]
        
        q0 = w
        q1 = x
        q2 = y
        q3 = z 

        roll = math.atan2(2.0 * (q3 * q2 + q0 * q1), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
        pitch = math.asin(2.0 * (q2 * q0 - q3 * q1))
        yaw = math.atan2(2.0 * (q3 * q0 + q1 * q2), -1.0 + 2.0 * (q0 * q0 + q1 * q1))

        return roll, pitch, yaw
 
def main(args=None):
    rclpy.init(args=args)
    arduino_node = Arduino()
    rclpy.spin(arduino_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
