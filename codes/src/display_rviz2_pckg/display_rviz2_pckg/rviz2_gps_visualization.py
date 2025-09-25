"""Node ROS 2 to display all the GPS-RTK in the system in RViz2."""

import rclpy
from rclpy.node import Node

from gps_msg_pckg.msg import SatMsgRcv
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped

from functools import partial
import math
import tf_transformations

class Rviz2GpsVisualization(Node):
    """Class of rviz2_gps_visualization node."""
    def __init__(self):
        """
        Constructor taking a dynamic parameter ROS 2 for agent names in the system using RTK GPS.

        :param agents_name: [ROS 2 param] List of agent names in the system, equipped with RTK GPS, defaults to ['']
        :type agents_name: list[str]
        """
        super().__init__('rviz2_gps_visualization')

        self.declare_parameter('agents_name', [''])
        list_agents = self.get_parameter('agents_name').get_parameter_value().string_array_value
        if list_agents == ['']:
            self.get_logger().warning('No agents name declared (empty list of str), node not used')
        else:
            self.list_subsc, self.dict_publisher = [], {}
            for agent in list_agents:
                self.list_subsc.append(self.create_subscription(SatMsgRcv, '/' + agent + '/topic_gps',
                                                                partial(self.listener_callback,
                                                                        topic_name=agent + '/topic_gps'), 1))
                self.list_subsc[-1]

                publisher_marker_pose = self.create_publisher(PoseStamped, 'marker_pose_' + agent, 1)
                publisher_marker_gps = self.create_publisher(Marker, 'marker_gps_' + agent, 1)
                publisher_marker_text = self.create_publisher(Marker, 'marker_text_' + agent, 1)
                publisher_marker_line = self.create_publisher(Marker, 'marker_line_' + agent, 1)

                # Not used, PoseStamped is better than an arrow marker
                # publisher_marker_course = self.create_publisher(Marker, 'topic_marker_course_' + agent, 1)

                self.dict_publisher[agent] = {'pose': publisher_marker_pose, 'gps': publisher_marker_gps,
                                              # 'course': publisher_marker_course,
                                              'text': publisher_marker_text,
                                              'line': publisher_marker_line, 'points': []}

    def listener_callback(self, msg_gps_rcv, topic_name):
        """
        Subscriber collecting the various GPS data, to create 4 publishers for RViz2.

        All our 4 publishers are in the same callback, so execution time is higher, but remains reasonable for display (5 ms).
        So there's no need to make multiple separate nodes in parallel.

        :param msg_gps_rcv: Message received in SatMsgRcv format
        :param topic_name: Name of our current subscriber's topic, allowing us to publish in the correct publishers
        :type topic_name: str
        :return: None
        """

        pub_topic = self.dict_publisher[topic_name.split('/')[0]]

        # == Arrow == (use PoseStamped is better)
        """print((msg_gps_rcv.course+90)%360)
        msg_course = Marker()
        msg_course.header.frame_id, msg_course.header.stamp = "world", self.get_clock().now().to_msg()
        msg_course.type, msg_course.action = 0, 0
        msg_course.pose.position.x, msg_course.pose.position.y, msg_course.pose.position.z = msg_gps_rcv.enu_east, msg_gps_rcv.enu_north, msg_gps_rcv.enu_up
        quat_arrow = tf_transformations.quaternion_from_euler(0, 0, math.radians(
            90 + msg_gps_rcv.course))  # Quaternion orienté à 0 vers axe x; sens trigo autour axe z
        msg_course.pose.orientation.x, msg_course.pose.orientation.y, msg_course.pose.orientation.z, msg_course.pose.orientation.w = \
            quat_arrow[0], quat_arrow[1], quat_arrow[2], quat_arrow[3]
        msg_course.scale.x, msg_course.scale.y, msg_course.scale.z = 1.0, 0.1, 0.1
        msg_course.color.r, msg_course.color.g, msg_course.color.b, msg_course.color.a = 0.0, 0.0, 1.0, 0.4
        pub_topic['course'].publish(msg_course)
        # self.get_logger().info('Publishing: "%s"' % msg_course)"""

        # == PoseStamped ==
        msg_pose = PoseStamped()
        msg_pose.header.frame_id, msg_pose.header.stamp = "world", self.get_clock().now().to_msg()
        msg_pose.pose.position.x, msg_pose.pose.position.y, msg_pose.pose.position.z = \
            msg_gps_rcv.enu_east, msg_gps_rcv.enu_north, msg_gps_rcv.enu_up
        quat_pose = tf_transformations.quaternion_from_euler(0, 0, math.radians(
            90 + msg_gps_rcv.course))  # Quaternion orienté à 0 vers axe x; sens trigo autour axe z
        msg_pose.pose.orientation.x, msg_pose.pose.orientation.y, msg_pose.pose.orientation.z, msg_pose.pose.orientation.w = \
            quat_pose[0], quat_pose[1], quat_pose[2], quat_pose[3]
        pub_topic['pose'].publish(msg_pose)
        # self.get_logger().info('Publishing: "%s"' % msg_course)

        # == Marker ellipsoid ==
        msg = Marker()
        msg.type, msg.action = 2, 0
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = msg_gps_rcv.enu_east, msg_gps_rcv.enu_north, msg_gps_rcv.enu_up
        msg.scale.x, msg.scale.y, msg.scale.z = 2 * msg_gps_rcv.sde, 2 * msg_gps_rcv.sdn, 2 * msg_gps_rcv.sdu
        red, green = 0.0, 0.0
        if msg_gps_rcv.status == 1: # Single (no RTK) solution
            red = 1.0
        elif msg_gps_rcv.status == 5: # Float solution
            red, green = 1.0, 0.5
        else: # Fix solution
            green = 1.0
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = red, green, 0.0, 0.7
        msg.header.frame_id, msg.header.stamp = "world", self.get_clock().now().to_msg()
        pub_topic['gps'].publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg)

        # == Text ==
        msg_text = Marker()
        msg_text.header.frame_id, msg_text.header.stamp = "world", self.get_clock().now().to_msg()
        msg_text.type, msg_text.action = 9, 0
        msg_text.pose.position.x, msg_text.pose.position.y, msg_text.pose.position.z = msg_gps_rcv.enu_east, msg_gps_rcv.enu_north, msg_gps_rcv.enu_up + 0.3  # display above 0.3 m
        msg_text.scale.z = 0.1
        msg_text.text = topic_name.split('/')[0] + '[' + str(round(msg_gps_rcv.speed * 1.852, 2)) + 'km/h]'
        msg_text.color.r, msg_text.color.g, msg_text.color.b, msg_text.color.a = 0.0, 0.0, 0.0, 0.5
        pub_topic['text'].publish(msg_text)
        # self.get_logger().info('Publishing: "%s"' % msg_text)

        # == Line ===
        msg_line = Marker()
        msg_line.header.frame_id, msg_line.header.stamp = "world", self.get_clock().now().to_msg()
        msg_line.type, msg_line.action = 4, 0
        msg_line.color.r, msg_line.color.g, msg_line.color.b, msg_line.color.a = 0.5, 0.5, 0.5, 0.5
        p = Point()
        p.x, p.y, p.z = msg_gps_rcv.enu_east, msg_gps_rcv.enu_north, msg_gps_rcv.enu_up
        pub_topic['points'].append(p)
        msg_line.points = pub_topic['points']
        msg_line.scale.x = 0.1
        pub_topic['line'].publish(msg_line)
        # self.get_logger().info('Publishing: "%s"' % msg_line)


def main(args=None):
    """ROS 2 node main."""
    rclpy.init(args=args)

    rviz2_gps_visualization = Rviz2GpsVisualization()

    rclpy.spin(rviz2_gps_visualization)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rviz2_gps_visualization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
