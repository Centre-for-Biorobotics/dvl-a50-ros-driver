#!/usr/bin/python3
"""
This script defines a ROS2 node named `dvl_serial_driver` that reads and publishes data
from a Water Linked A50 DVL (Doppler Velocity Log) device connected via a serial port.

The node publishes the DVL data to the `/dvl/data` topic using the custom DVL message type.
"""

import rclpy
from rclpy.node import Node

from waterlinked_a50_interfaces.msg import DVL, DVLBeam
from std_msgs.msg import String

# import wldvl library from https://github.com/waterlinked/dvl-python
# installed according to https://github.com/waterlinked/dvl-python/blob/master/serial/README.md
from wldvl import WlDVL


class DVLSerialDriver(Node):
    """
    A ROS2 node that interfaces with a Water Linked A50 DVL device.
    """

    def __init__(self):
        """
        Initializes the DVLSerialDriver node.

        Reads the 'serial_port' parameter and sets up the DVL device communication.
        Initializes a publisher and a timer for periodic data reads.
        """
        super().__init__('dvl_serial_driver')
        
        # declare and get the 'serial_port' parameter. Default value is '/dev/ttyUSB0'.
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.serial_port = self.get_parameter('serial_port').value

        self.get_logger().info(f'DVL at port: {self.serial_port}')

        # set up the DVL communication
        self.dvl = WlDVL(self.serial_port)

        # self.get_logger().info(f'Connected: {self.dvl}')

        # initialize a publisher to `/dvl/data` topic.
        self.dvl_pub = self.create_publisher(DVL, '/dvl/data', 10)

        # set up a timer for periodic data acquisition 
        self.pub_timer_period = 1  # (s)
        self.pub_timer = self.create_timer(
            self.pub_timer_period,
            self.getData
        )

    def getData(self):
        """
        Reads data from the DVL device and publishes if the data is valid.
        """
        report = self.dvl.read()
        self.get_logger().info(f'report: {report}')
        # self.get_logger().info(f"Velocity , {report['vx']}, {report['vx']}, {report['vz']}")
        if report is not None:
            self.publish(report)

    def publish(self, report):
        """
        Publishes the DVL data to the `/dvl/data` topic.

        Args:
            report (dict): A dictionary containing the DVL data.
        """
        theDVL = DVL()

        theDVL.header.stamp = self.get_clock().now().to_msg()
        theDVL.header.frame_id = "a50_dvl"
        theDVL.velocity.x = report['vx']
        theDVL.velocity.y = report['vy']
        theDVL.velocity.z = report['vz']
        theDVL.fom = report['fom']
        theDVL.altitude = report['altitude']
        theDVL.velocity_valid = report['valid']

        self.dvl_pub.publish(theDVL)



def main(args=None):
    """
    Main entry point for the script.
    """
    rclpy.init(args=args)
    dvl_serial_driver = DVLSerialDriver()
    rclpy.spin(dvl_serial_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dvl_serial_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
