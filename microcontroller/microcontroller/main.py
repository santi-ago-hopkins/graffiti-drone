import rclpy
import numpy as np
import re
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu


class Arduino(Node):
    def __init__(self):
        super().__init__('arduino_connection_node')

        self.control_subscriber = self.create_subscription(
            Float64,
            '/cmd',
            self.cmd_callback,
            1
        )

        # IMU Data Publisher
        self.imu_publisher = self.create_publisher(
            Imu,
            '/imu',
            1
        )
        
        # Initialize controls
        self.acceleration = 0
        self.steering_angle = 0 
        self.is_serial_communication_active = True
        self.serial_port = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_6493433313535110B121-if00'  # Update this with serial port if changes
        self.baud_rate = 115200
        self.ser = ser = serial.Serial(self.serial_port, self.baud_rate)

        # IMU Publishing Bits and Bobs
        self.orientation_ready = False
        self.acceleration_ready = False
        self.x_orientation = 0.0
        self.y_orientation = 0.0
        self.z_orientation = 0.0
        self.x_acceleration = 0.0
        self.y_acceleration = 0.0
        self.z_acceleration = 0.0
        
        # Create a timer that goes off every 0.001 seconds, and calls self.recieve_serial_values
        # VERY ABITRARILY CREATED, SOMEBODY WRITE THIS BETTER
        self.serial_timer = self.create_timer(0.001, self.recieve_serial_values)


    def cmd_callback(self, msg: AckermannDriveStamped):
        """ 
        Callback function that should recieve car commands from MPC.
        Sends the acceleration and steering values to the arduino through Serial.
        """
        steering_value = msg.drive.steering_angle
        print("recieved message: ", steering_value)
        #Convert the steering value to what the motor has to rotate
        motor_value = -1 * np.round(np.clip(steering_value * 4.615, (-2 *np.pi), (2 * np.pi)), 2)
        motor_str = str(motor_value) + '\n'
        motor_float = float(motor_str)
        print("Motor Float: ", motor_float)
        #self.ser.write(str(motor_str).encode('utf-8'))
        #self.ser.write(motor_str.encode())
        self.ser.write(bytearray(motor_str, 'ascii'))
        print("sent message")
        return 

    def recieve_serial_values(self):
        #Note: This probably isn't the most time efficient method of doing this, but I just wrote this in a day for push-testing
        if self.ser.in_waiting > 0: 
            data = self.ser.readline()

            # Different possible messages we could recieve over serial
            orientation_pattern = re.compile(r'X Orientation: ([\d.-]+) Y Orientation: ([\d.-]+) Z Orientation: ([\d.-]+)')
            acceleration_pattern = re.compile(r'X Acceleration: ([\d.-]+) Y Acceleration: ([\d.-]+) Z Acceleration: ([\d.-]+)')
            velocity_pattern = re.compile(r'Velocity: ([\d.-]+)')

            # Check which match we have

            # Case 1: Orientation Data
            orientation_match = orientation_pattern.search(str(data))
            if orientation_match:
                self.x_orientation = float(orientation_match.group(1))
                self.y_orientation = float(orientation_match.group(2))
                self.z_orientation = float(orientation_match.group(3))
                #print("X Orientation: ", self.x_orientation)
                self.orientation_ready = True

            # Case 2: Acceleration Data
            acceleration_match = acceleration_pattern.search(str(data))
            if acceleration_match:
                self.x_acceleration = float(acceleration_match.group(1))
                self.y_acceleration = float(acceleration_match.group(2))
                self.z_acceleration = float(acceleration_match.group(3))
                #print("X Accel: ", self.x_acceleration)
                #print("Y Accel: ", self.y_acceleration)
                #print("Z Accel: ", self.z_acceleration)
                self.acceleration_ready = True

            # Case 3: Velocity Data
            velocity_match = velocity_pattern.search(str(data))
            if velocity_match: 
                msg = Float64()
                if (float(velocity_match.group(1)) < 0 ):
                    return
                msg.data = float(velocity_match.group(1))
                self.wheelspeed_publisher.publish(msg)
                #self.get_logger().info(f"Published Wheelspeed data: {msg}")
                
            if self.orientation_ready is True and self.acceleration_ready is True: 
                # Publish IMU message
                imu_msg = Imu()

                # Fill orientation (converting degrees to radians)
                imu_msg.orientation.x = np.radians(self.x_orientation)
                imu_msg.orientation.y = np.radians(self.y_orientation)
                imu_msg.orientation.z = np.radians(self.z_orientation)

                # Fill linear acceleration
                imu_msg.linear_acceleration.x = self.x_acceleration
                imu_msg.linear_acceleration.y = self.y_acceleration
                imu_msg.linear_acceleration.z = self.z_acceleration
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                self.imu_publisher.publish(imu_msg)
                #self.get_logger().info(f"Published IMU data: {imu_msg}")
    
def main(args=None):
    rclpy.init(args=args)
    arduino_node = Arduino()
    rclpy.spin(arduino_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    