import Adafruit_BMP.BMP085 as BMP085

import numpy as np
import rclpy
from rclpy.node import Node


from std_msgs.msg import Float32
from sensor_msgs.msg import Temperature, FluidPressure

class BMP180_NODE(Node):

    def __init__(self):
        super().__init__('bmp180')


        #Default Parameters
        self.rate = 20

        self.pressure_topic = "/ahrs/imu/accelerometer"
        self.altitude_topic = "/ahrs/imu/magnetometer"
        self.temperature_topic = "/ahrs/imu/angular_rates"    


        self.declare_parameter('data_query_frequency', value=self.rate)

        self.declare_parameter('ros_topic_altitude', value=self.altitude_topic)
        self.declare_parameter('ros_topic_temperature', value=self.temperature_topic)
        self.declare_parameter('ros_topic_pressure', value=self.pressure_topic)


        try:
            self.rate = self.get_parameter('data_query_frequency').value
            self.get_logger().info('Obtained "%s" and its value is "%s"' %('data_query_frequency', str(self.rate)))

            self.altitude_topic = self.get_parameter('ros_topic_altitude').value
            self.get_logger().info('Obtained "%s" and its value is "%s"' %('ros_topic_altitude', str(self.altitude_topic)))

            self.temperature_topic = self.get_parameter('ros_topic_temperature').value
            self.get_logger().info('Obtained "%s" and its value is "%s"' %('ros_topic_temperature', str(self.temperature_topic)))

            self.pressure_topic = self.get_parameter('ros_topic_pressure').value
            self.get_logger().info('Obtained "%s" and its value is "%s"' %('ros_topic_pressure', str(self.pressure_topic)))


        except Exception as e:
            node.get_logger().warn('Could not get parameters! Default parameters will be used!')
 

        self.altitude = Float32()
        self.temperature = Temperature()
        self.pressure = FluidPressure()

        self.bmp = BMP085.BMP085()

        #Initialization
        self._altitude_pub = self.create_publisher(Float32, self.altitude_topic, 4)
        self._temperature_pub = self.create_publisher(Temperature, self.temperature_topic, 4)
        self._pressure_pub = self.create_publisher(FluidPressure, self.pressure_topic, 4)
        self._bmp180_timer = self.create_timer(1/self.rate, self._bmp180_timer_handler)


    def _bmp180_timer_handler(self):
        self.temperature.temperature = self.bmp.read_temperature()
        self.pressure.fluid_pressure = float(self.bmp.read_pressure())
        self.altitude.data = self.bmp.read_altitude()

        self._temperature_pub.publish(self.temperature)
        self._pressure_pub.publish(self.pressure)
        self._altitude_pub.publish(self.altitude)



def main():
    rclpy.init()
    bmp180 = BMP180_NODE()
    bmp180.get_logger().info('BMP180 Node HAS BEEN ACTIVATED!')
    rclpy.spin(bmp180)


if __name__ == '__main__':
    main()
