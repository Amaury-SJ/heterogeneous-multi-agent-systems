"""Node ROS 2 for calculating the 3D Euclidean distance between two RTK GPS positions."""

import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from gps_msg_pckg.msg import SatMsgRcv
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

import math


class CalculateDistGps(Node):
    """Class of calculate_dist_gps node."""
    def __init__(self):
        """
        Constructor taking as dynamic parameters ROS 2 the two GPS names for distance estimation.
        
        :param first_gps: [ROS 2 param] Name of first GPS, defaults to 'first_gps'
        :type first_gps: str
        :param second_gps: [ROS 2 param] Name of second GPS, defaults to 'second_gps'
        :type second_gps: str
        """
        super().__init__('calculate_dist_gps')

        self.declare_parameter('first_gps', 'first_gps')
        self.declare_parameter('second_gps', 'second_gps')
        self.first_gps = self.get_parameter('first_gps').get_parameter_value().string_value
        self.second_gps = self.get_parameter('second_gps').get_parameter_value().string_value

        self.first_gps_sub = Subscriber(self, SatMsgRcv, self.first_gps + '/topic_gps')
        self.second_gps_sub = Subscriber(self, SatMsgRcv, self.second_gps + '/topic_gps')
        queue_size, delay = 1, 1
        self.publisher_distance = self.create_publisher(Float64, 'd_' + self.first_gps + '_' + self.second_gps, 1)
        self.ts = ApproximateTimeSynchronizer([self.first_gps_sub, self.second_gps_sub], queue_size, delay)
        self.ts.registerCallback(self.multi_subsc)

    def multi_subsc(self, first_gps_msg, second_gps_msg):
        """Multiple subscriber with two topics (SatMsgRcv type) and approximate time synchronization, and publish the result (float type)."""
        #To calculate with the errors in 3D
        #msg = Vector3()
        # num = 2 * (first_gps_msg.sde - second_gps_msg.sde) / (first_gps_msg.enu_east - second_gps_msg.enu_east) + 2 * (
        #         first_gps_msg.sdn - second_gps_msg.sdn) / (first_gps_msg.enu_north - second_gps_msg.enu_north) + 2 * (
        #               first_gps_msg.sdu - second_gps_msg.sdu) / (first_gps_msg.enu_up - second_gps_msg.enu_up)
        # deno = (first_gps_msg.enu_east - second_gps_msg.enu_east) ** 2 + (
        #             first_gps_msg.enu_north - second_gps_msg.enu_north) ** 2 + (
        #                    first_gps_msg.enu_up - second_gps_msg.enu_up) ** 2
        # delta = abs(0.5 * num / deno)
        # msg.x, msg.y, msg.z = value + delta, value, value - delta

        msg = Float64()
        value = math.sqrt((first_gps_msg.enu_east - second_gps_msg.enu_east) ** 2 + (
                first_gps_msg.enu_north - second_gps_msg.enu_north) ** 2 +
                          (first_gps_msg.enu_up - second_gps_msg.enu_up) ** 2)
        msg.data = value - 0.9 # We want only the error

        self.publisher_distance.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % value)


def main(args=None):
    """ROS 2 node main."""
    rclpy.init(args=args)

    Calculate_Dist_Gps = CalculateDistGps()

    rclpy.spin(Calculate_Dist_Gps)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Calculate_Dist_Gps.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
