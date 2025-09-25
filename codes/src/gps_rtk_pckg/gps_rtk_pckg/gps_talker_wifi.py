import rclpy
from rclpy.node import Node
from gps_msg_pckg.msg import SatMsgRcv

import socket
import time


class GpsTalker(Node):
    """Class of gps_talker_wifi node."""
    def __init__(self):
        """
        Constructor taking dynamics parameters ROS 2 for the update rate and the ip name via Wi-Fi.
        Publish a topic with all the data received.
        IMPORTANT: We recommand to receive data by USB, with the other node.

        :param update_rate_hz: [ROS 2 param] Update rate of data reception. For Emlid Reach M2, defaults to 10.0 Hz.
        :type update_rate_hz: float
        :param ip_host: [ROS 2 param] The IP adress via Wi-Fi, defaults to '192.168.42.1' in access point for the GPS.
        :type ip_host: str
        :param port_nmea: [ROS 2 param] Port name to receive NMEA data, defaults to 9001.
        :type port_nmea: int
        :param port_enu: [ROS 2 param] Port name to receive ENU data, defaults to 9002.
        :type port_enu: int
        """
        super().__init__('gps_talker_wifi')

        # Parameters
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('ip_host', '192.168.42.1')
        self.declare_parameter('port_nmea', 9001)	
        self.declare_parameter('port_enu', 9002)

        self.freq = self.get_parameter('update_rate_hz').get_parameter_value().double_value
        self.host = self.get_parameter('ip_host').get_parameter_value().string_value
        self.port_nmea = self.get_parameter('port_nmea').get_parameter_value().integer_value
        self.port_enu = self.get_parameter('port_enu').get_parameter_value().integer_value

        self.client_nmea, self.client_enu = None, None

        self.connect_wifi_nmea()
        self.connect_wifi_enu()

        self.publisher_ = self.create_publisher(SatMsgRcv, 'topic_gps', 1)
        timer_period = 0.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Callback function called to receive each RTK GPS trame.
        We will receive the NMEA data, then the ENU data, if the connection is not lost.
        If the data is not empty, we'll identify the useful data,
        retrieve them and publish them in a topic.

        :return: None
        """
        self.msg = SatMsgRcv()
        raw_data_nmea, raw_data_enu = '', ''

        try:
            raw_data_nmea = self.client_nmea.recv(1024)
            # print('== ', raw_data_nmea)
        except socket.error:
            self.get_logger().info("Connection lost... reconnecting " + self.host + ':' + str(self.port_nmea))
            self.connect_wifi_nmea()

        try:
            raw_data_enu = self.client_enu.recv(1024)
            # print(raw_data_enu)
        except socket.error:
            self.get_logger().info("Bad signal... reconnecting " + self.host + ':' + str(self.port_enu))
            self.connect_wifi_enu()

        if raw_data_nmea != '' and raw_data_enu != '':
            extrac_nmea = self.extraction_donnees_gps_nmea(raw_data_nmea)
            extarc_enu = self.extraction_donnees_gps_enu(raw_data_enu)
            if extrac_nmea and extarc_enu:
                self.msg.header.frame_id, self.msg.header.stamp = "world", self.get_clock().now().to_msg()
                self.publisher_.publish(self.msg)
                self.get_logger().info('Publishing: "%s"' % self.msg.status)
                print(self.msg)

    def connect_wifi_nmea(self):
        """
        Function for reconnecting via the NMEA port in Wi-Fi.

        :return: None
        """
        start_time = time.time()
        while True:
            self.client_nmea = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_nmea.settimeout(2 / self.freq)  # Commands are blocking for X sec
            try:
                self.client_nmea.connect((self.host, self.port_nmea))
                self.get_logger().info('Connection to ' + self.host + ':' + str(self.port_nmea) + ' successful')
                break
            except socket.error:
                downtime = time.time() - start_time
                self.get_logger().warn('Fail to connect to ' + self.host + ':' + str(self.port_nmea) + ' during ' + str(
                    round(downtime, 1)) + ' s')
                time.sleep(2 / self.freq)

    def connect_wifi_enu(self):
        """
        Function for reconnecting via the ENU port in Wi-Fi.

        :return: None
        """
        start_time = time.time()
        while True:
            self.client_enu = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_enu.settimeout(2 / self.freq)  # Commands are blocking for X sec
            try:
                self.client_enu.connect((self.host, self.port_enu))
                self.get_logger().info('Connection to ' + self.host + ':' + str(self.port_enu) + ' successful')
                break
            except socket.error:
                downtime = time.time() - start_time
                self.get_logger().warn('Fail to connect to ' + self.host + ':' + str(self.port_enu) + ' during ' + str(
                    round(downtime, 1)) + ' s')
                time.sleep(2 / self.freq)

    def extraction_donnees_gps_nmea(self, raw_d):
        """
        Extracts the desired data from the raw data received via the NMEA port.

        :param raw_d: Raw data received via the NMEA port.
        :type raw_d: str
        :return: bool
        """
        data = str(raw_d).split(",")  # Receive and split data
        if data[0] == "b'$GNRMC" and data[1] != '' and len(raw_d) < 200:
            # Conditions to check if no offset and data received and if not an old frame

            lat_deg_min, long_deg_min = float(data[3]), float(data[5])
            self.msg.latitude, self.msg.longitude = round(lat_deg_min // 100 + (lat_deg_min % 100) / 60, 9), round(
                long_deg_min // 100 + (long_deg_min % 100) / 60, 9)
            self.msg.altitude = float(data[21])
            self.msg.speed = float(data[7])
            if data[8] != '':
                self.msg.course = 360 - float(data[8]) # To obtain trigonometric direction
            return True
        else:
            self.get_logger().info('Data corrupted')
            return False

    def extraction_donnees_gps_enu(self, raw_d):
        """
        Extracts the desired data from the raw data received via the ENU port.

        :param raw_d: Raw data received via the ENU port.
        :return: bool
        """
        # COndition not test with 'b' but apply a decode then test that we have a '2' from 2022
        if str(raw_d)[0] == 'b' and len(raw_d) < 200:  # avoid offsets and old frames
            data = str(raw_d).split(' ')
            new_data = []
            for element in data:
                if element != '':
                    new_data.append(element)
            self.msg.timer = new_data[0][2:] + ' ' + new_data[1]
            self.msg.enu_east, self.msg.enu_north, self.msg.enu_up = float(new_data[2]), float(new_data[3]), float(
                new_data[4])
            self.msg.status = int(new_data[5])
            self.msg.sdn, self.msg.sde, self.msg.sdu = float(new_data[7]), float(new_data[9]), float(new_data[8])
            return True
        else:
            self.get_logger().info('Data corrupted')
            return False


def main(args=None):
    """ROS 2 node main."""
    rclpy.init(args=args)

    gps_talker = GpsTalker()

    rclpy.spin(gps_talker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
