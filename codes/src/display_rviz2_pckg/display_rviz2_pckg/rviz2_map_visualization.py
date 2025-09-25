"""Node ROS 2 to display the map in RViz2."""

import rclpy
from rclpy.node import Node

from gps_msg_pckg.msg import SatMsgRcv
from visualization_msgs.msg import Marker

import math
import os
import shutil

from geographiclib.geodesic import Geodesic

geod = Geodesic.WGS84  # define the WGS84 ellipsoid


class Rviz2MapVisualization(Node):
    """Class of rviz2_map_visualization node."""
    def __init__(self):
        """
        Constructor taking a dynamic parameter ROS 2 for agent names in the system using RTK GPS.

        :param name_map: [ROS 2 param] Name of the map you wish to display, defaults to 'map_plaine'
        :type name_map: str
        :param reference_agent_name: [ROS 2 param] Name of one agent of the system, to calculate the position of our map in 3D space, defaults to 'human_1'
        :type reference_agent_name: str
        :param antenna_height: [ROS 2 param] If you want t ouse your own RTK base as origin and fix the altitude of the map, defaults to 0.065
        :type antenna_height: float
        """
        super().__init__('rviz2_map_visualization')

        self.declare_parameter('name_map', 'map_plaine') # Select the map we want
        self.declare_parameter('reference_agent_name', 'human_1')
        self.declare_parameter('antenna_height', 0.065)  # Size of a RS+ between antenna and ground, as RTK base
        self.map_altitude = 9000.0 # To have the map displaying always bellow all objects. High random number max earth

        self.reference_agent_name = self.get_parameter('reference_agent_name').get_parameter_value().string_value
        self.antenna_height = self.get_parameter('antenna_height').get_parameter_value().double_value
        self.name_picture_map = self.get_parameter('name_map').get_parameter_value().string_value

        # == Select the corresponding image ==
        # Select the corresponding image for map.png, ressource use for our .dae model
        os.remove(os.getcwd() + '/src/display_rviz2_pckg/images/map.png')
        shutil.copy2(os.getcwd() + '/src/display_rviz2_pckg/images/' + self.name_picture_map + '.png',
                     os.getcwd() + '/src/display_rviz2_pckg/images/map.png')

        # == Display Map ==
        self.subsc = self.create_subscription(SatMsgRcv, '/' + self.reference_agent_name + '/topic_gps',
                                              self.listener_gps_for_publish_map, 1)
        self.subsc
        self.publisher_marker_map = self.create_publisher(Marker, 'marker_map', 1)

    def listener_gps_for_publish_map(self, msg_gps_rcv):
        """
        Subscriber to the GPS data of one agent, calculate the position of the local map, and publish the marker.

        :param msg_gps_rcv: Message received in SatMsgRcv format
        :return: None
        """
        if msg_gps_rcv.status == 1 or 4 or 5:
            coordinate_map = self.select_coordinate_map(self.name_picture_map)

            msg = Marker()
            msg.header.frame_id, msg.header.stamp = "world", self.get_clock().now().to_msg()
            msg.type, msg.action = 10, 0

            # == Offsets ==
            lat_middle_map, long_middle_map = coordinate_map[2] + (coordinate_map[0] - coordinate_map[2]) / 2, \
                                              coordinate_map[1] + (coordinate_map[3] - coordinate_map[1]) / 2
            dist_lat = geod.Inverse(lat_middle_map, long_middle_map, msg_gps_rcv.latitude, long_middle_map)['s12']
            dist_long = geod.Inverse(lat_middle_map, long_middle_map, lat_middle_map, msg_gps_rcv.longitude)['s12']
            sign_lat = math.copysign(1, lat_middle_map - msg_gps_rcv.latitude)
            sign_long = math.copysign(1, long_middle_map - msg_gps_rcv.longitude)
            res_y = sign_lat * dist_lat + msg_gps_rcv.enu_north
            res_x = sign_long * dist_long + msg_gps_rcv.enu_east
            msg.pose.position.x, msg.pose.position.y = res_x, res_y

            # == Altitude ==
            # Possible to use our own RTK base as the origin
            # msg.pose.position.z = -self.antenna_height
            # With the minimal altitude position of our agent
            self.map_altitude = min(self.map_altitude,msg_gps_rcv.enu_up)
            msg.pose.position.z = self.map_altitude

            # == Scale ==
            # On peut faire le calcul depuis la base avec pymap3d to convert LLH to ENU
            width_in_m = geod.Inverse(coordinate_map[0], coordinate_map[1], coordinate_map[0], coordinate_map[3])['s12']
            height_in_m = geod.Inverse(coordinate_map[0], coordinate_map[1], coordinate_map[2], coordinate_map[1])[
                's12']
            msg.scale.x, msg.scale.y, msg.scale.z = width_in_m, height_in_m, 0.0

            # == Mesh file ==
            msg.mesh_resource = 'file://' + os.getcwd() + '/src/display_rviz2_pckg/images/object_map.dae'
            # msg.mesh_resource = 'file://' + os.getcwd() + '/src/display_rviz2_pckg/images/' + self.name_picture_map + '.dae'
            # 'file:///home/vm-amaury/dev_ws/src/display_rviz2_pckg/images/map_3d.dae'
            msg.mesh_use_embedded_materials = True

            self.publisher_marker_map.publish(msg)
            # self.get_logger().info('Publishing: "%s"' % msg)
        else:
            self.get_logger().warning('Impossible to display the map: GPS data empty from ' + self.reference_agent_name)

    def select_coordinate_map(self, name):
        """
        Function to find out the dimensions of the selected tile.

        Format for adding a map: 'name_map': [top latitude, left longitude, bottom latitude, right longitude]

        :param name: Name of the selected map
        :type name: str
        :return: Coordinates of the four sides of the selected map
        :rtype: list[float]
        """
        dic_map = {'map_plaine': [48.67344, 6.16684, 48.67182, 6.17038],
                   'map_potager': [48.67494, 6.16721, 48.67332, 6.17075],
                   'map_bat_parrot': [48.87969, 2.36606, 48.87808, 2.36960],
                   'map_base_aerienne': [48.57492, 5.94362, 48.57330, 5.94716],
                   'map_loria': [48.66848, 6.14926, 48.66201, 6.16343],
                   'map_nancy': [48.7256, 6.0723, 48.6221, 6.2989],
                   'map_antony': [48.75439, 2.30935, 48.75278, 2.31270]}

        return dic_map[name]


def main(args=None):
    """ROS 2 node main."""
    rclpy.init(args=args)

    rviz2_map_visualization = Rviz2MapVisualization()

    rclpy.spin(rviz2_map_visualization)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rviz2_map_visualization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
            # Other method in listener_gps_for_publish_map pour afficher la carte à l'aide de nuage de points à la place du mesh
            #msg.color.a = 1.0 # je ne sais plus si besoin ici
            #msg.color.r,msg.color.g,msg.color.b = 0.0,1.0,0.0
            import matplotlib.image as mpimg
            from std_msgs.msg import ColorRGBA
            from geometry_msgs.msg import Point
            img = mpimg.imread('src/display_rviz2_pckg/images/' +'map_antony.png')
            l_colors,l_points=[],[]
            for i in range(0,len(img)):
                for j in range(0,len(img[0])):
                    c = ColorRGBA()
                    c.r, c.g, c.b, c.a = float(img[i][j][0]), float(img[i][j][1]), float(img[i][j][2]), float(img[i][j][3])
                    l_colors.append(c)
                    p =Point()
                    p.x, p.y, p.z = float(i)/10,float(j)/10,0.0
                    l_points.append(p)
            msg.colors,msg.points = l_colors,l_points"""
