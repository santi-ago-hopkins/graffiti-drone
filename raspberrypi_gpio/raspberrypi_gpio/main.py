import rcply 
import time
import board
import adafruit_bno055
from sensors_msgs.msg import Imu

class PiGPIO(Node):
    def __init__(self):
        super().__init__('raspberry_pi_GPIO_node')

        self.imu_publisher = self.create_publisher(
                IMU,
                '/imu',
                1
                )

        i2c = board.I2C()  # uses board.SCL and board.SDA
        # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
        sensor = adafruit_bno055.BNO055_I2C(i2c)
        last_val = 0xFFFF
        self.serial_timer = self.create_timer(0.001, self.recieve_values)


    def recieve_values(self):
        #print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
        #print("Gyroscope (rad/sec): {}".format(sensor.gyro))
        #print("Euler angle: {}".format(sensor.euler))
        #print("Quaternion: {}".format(sensor.quaternion))
        #print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
        #print("Gravity (m/s^2): {}".format(sensor.gravity))
        #print()
        msg = Imu()
        #msg.orientation.x = sensor.
        #msg.orientation.y = sensor.
        #msg.orientation.z = sensor.
        #msg.orientation.w = sensor.

        #msg.acceleration.x
        #msg.acceleration.y
        #msg.acceleration.z
        msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_publisher.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    pi_node = PiGPIO()
    rclpy.spin(pi_node)
    rcply.shutdown()

if __name__ == '__main__':
    main()
