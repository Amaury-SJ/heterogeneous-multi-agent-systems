import rclpy
from rclpy.node import Node
from gps_msg_pckg.msg import SatMsgRcv
from gps_msg_pckg.srv import BasePosition

import serial
import time
import pymap3d

class GpsTalkerUsb(Node):
    """Class of gps_talker node."""
    def __init__(self):
        """
        Constructor taking dynamics parameters ROS 2 for the update rate and the device name via USB.
        Publish a topic with all the data received.

        :param update_rate_hz: [ROS 2 param] Update rate of data reception. For Emlid Reach M2, defaults to 10.0 Hz.
        :type update_rate_hz: float
        :param device_name: [ROS 2 param] The port USB name, defaults to '/dev/ttyACM0'.
        :type device_name: str
        """
        super().__init__('gps_talker')

        # Parameters
        self.declare_parameter('update_rate_hz', 10.0)
        self.freq = self.get_parameter('update_rate_hz').get_parameter_value().double_value
        self.declare_parameter('device_name',"/dev/ttyACM0")
        self.device_name = self.get_parameter('device_name').get_parameter_value().string_value

        self.publisher = self.create_publisher(SatMsgRcv, 'topic_gps', 1)
        timer_period = 0.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.connection()
        self.base_position_srv = self.create_service(BasePosition, 'base_position', self.base_position_srv_callback)
        self.base_position_saved = [0.0,0.0,0.0]


    def base_position_srv_callback(self, request, response):
        """
        Work in progress.

        :param request:
        :param response:
        :return: None
        """
        response.base_latitude, response.base_longitude, response.base_altitude = self.base_position_saved
        self.get_logger().info('Service base position request incoming: ' + str(self.base_position_saved))
        return response

    def timer_callback(self):
        """
        Callback function to read the USB port, and retrieve data, if there is no connection error.
        Useful data is extracted from the raw data, then published in a topic with the correct message format.

        :return: None
        """
        self.msg = SatMsgRcv()
        raw_data_nmea = ''
        try:
            # 'read' command is a blocking command with the timeout defined in the port
            raw_data_nmea = self.port.read(10000)
            #print('== Raw data:', raw_data_nmea)

        except Exception as error_read:
            # Error returned:
            # device reports readiness to read but returned no data (device disconnected or multiple access on port?)
            self.get_logger().error(error_read)
            self.connection()

        if raw_data_nmea != '':
            extraction_nmea = self.extraction_donnees_gps_nmea(raw_data_nmea)
            if extraction_nmea:
                self.msg.header.frame_id, self.msg.header.stamp = "world", self.get_clock().now().to_msg()
                self.publisher.publish(self.msg)
                self.get_logger().info('Publishing...')
                print(self.msg)

    def connection(self):
        """
        Function to define and restore USB connection in the case of an error. WARNING: For a frequency of 100ms, we read the port all the 10ms (10%).

        :return: None
        """
        connected = False
        while not connected:
            try: # 10 % of the frequency to actualize
                self.port = serial.Serial(self.device_name, baudrate=115200, timeout=1 / (self.freq * 10))
                connected = True
            except Exception as error_connection:
                # Error returned:
                # [Errno 2] could not open port /dev/ttyACM0: [Errno 2] No such file or directory: '/dev/ttyACM0'
                self.get_logger().error(str(error_connection))
                time.sleep(1 / self.freq)

    def extraction_donnees_gps_nmea(self, raw_d):
        """
        Extracts the desired data from the raw data received via the USB port, in NMEA format.

        :param raw_d:
        :type raw_d: str
        :return:
        """
        data = str(raw_d).split(",")  # Receive and split data
        if data[0] != "b''": # If USB data empty
            if data[1] != '': # If GPS signal received
                if data[0] == "b'$GNRMC": # If no shift
                    if len(raw_d) < 270: # If old frames (lenght for a frame: RMC, GGA, GST, EBP)
                        lat_deg_min, long_deg_min = float(data[3]), float(data[5])
                        # Improvement: for locations other than data[4]=North and data[6]=East, add their signs
                        self.msg.latitude, self.msg.longitude = round(lat_deg_min // 100 + (lat_deg_min % 100) / 60, 9), round(
                            long_deg_min // 100 + (long_deg_min % 100) / 60, 9)
                        self.msg.altitude = float(data[21])
                        self.msg.status = int(data[18])
                        self.msg.speed = float(data[7])
                        if data[8] != '': # Need a first movement to initialize
                            self.msg.course = 360 - float(data[8]) # To obtain sens trigo
                        self.msg.timer = str(data[1]) + ' ' + str(data[9])
                        self.msg.sdn, self.msg.sde, self.msg.sdu = float(data[32]), float(data[33]), float(data[34].split('*')[0])
                        if data[35] != '':
                            base_lat_deg_min, base_long_deg_min = float(data[35]), float(data[37])
                            self.msg.base_latitude, self.msg.base_longitude = round(base_lat_deg_min // 100 + (base_lat_deg_min % 100) / 60, 9), round(
                            base_long_deg_min // 100 + (base_long_deg_min % 100) / 60, 9)
                            self.msg.base_altitude = float(data[39])
                            self.base_position_saved = [self.msg.base_latitude, self.msg.base_longitude, self.msg.base_altitude]
                            self.msg.enu_east, self.msg.enu_north, self.msg.enu_up = pymap3d.geodetic2enu(
                                self.msg.latitude, self.msg.longitude, self.msg.altitude, self.msg.base_latitude, self.msg.base_longitude, self.msg.base_altitude)
                            return True
                        else:
                            self.get_logger().info('No RTK: No base coordinates received')
                            return False
                    else:
                        self.get_logger().warning('GPS data corrupted (old/multiples frames)')
                        return False
                else:
                    self.get_logger().warning('GPS data corrupted (shift)')
                    return False
            else:
                self.get_logger().warning('GPS data corrupted (no GPS signal)')
                return False
        else:
            self.get_logger().warning('No USB data (empty) but IGNORE (because of update rate higher)')
            return False


def main(args=None):
    """ROS 2 node main."""
    rclpy.init(args=args)

    gps_talker_usb = GpsTalkerUsb()

    rclpy.spin(gps_talker_usb)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_talker_usb.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
