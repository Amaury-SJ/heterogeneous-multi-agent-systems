"""Node ROS 2 to display the type of solution for the two GPS units, and their relative distances.
The distance between the two gps during the two experiements isn't really good, because of the inside gps.
"""

import rclpy
from rclpy.node import Node
from gps_msg_pckg.msg import SatMsgRcv
from std_msgs.msg import Float64
from geographiclib.geodesic import Geodesic
import math
geod = Geodesic.WGS84  # define the WGS84 ellipsoid


class GpsSolStatusAndDistance(Node):
    """Class of gps_sol_status_and_distance node."""
    def __init__(self):
        """
        Constructor taking as dynamic parameter ROS 2 the name of the GPS on the car roof, to subscribe .

        """
        super().__init__('gps_sol_status_and_distance')

        # First: GNSSM2_raw_20240325135850.pos and second: GNSSM2_raw_20240325143439.pos
        file = open("GNSSM2_raw_20240325135850.pos",'r')
        # Time spent before starting:
        lines_to_pass = 1876 # 1876 lines passed for first_exp or 776 lines passed for second_exp.
        self.temporal_shift = 178
        # We have a temporal drift of 178 for the first, and not estimated for the second, because not pertinent.

        # == Traitement du fichier ==
        self.list_data = []
        self.status_inside_dict = {'Single': 0, 'Fix': 0, "Float": 0}
        first_lines = 0
        # Read the 10 first lines (file header), and for synchronization 1876 lines passed for first_exp.
        # And 776 lines passed for second_exp.
        while first_lines < 10 + lines_to_pass :
            file.readline()
            first_lines +=1

        for line in file:
            self.list_data.append(line.split())
            Q_value = line.split()[5] # Format LLH from RTKlib: 1 fix, 2 float, 5 single
            if Q_value == '1':
                self.status_inside_dict['Fix'] += 1
            elif Q_value == '2':
                self.status_inside_dict['Float'] += 1
            elif Q_value == '5':
                self.status_inside_dict['Single'] += 1
            else :
                print('Strange value: ', Q_value)
        print('Solution status for GPS inside the car: ', self.status_inside_dict)
        print('Lenght data file: ', len(self.list_data))
        self.count = 0
        # ===========================

        self.solutions_dict = {'Single': 0, 'Fix': 0, 'Float': 0, 'Lost': 0, 'Lost of inside GPS':0} # Solution status: 1 Single (no RTK), 4 (Fix), 5 (Float)
        self.line_nbr =0

        self.declare_parameter('gps_name', 'human_1')
        self.gps_name = self.get_parameter('gps_name').get_parameter_value().string_value

        self.publisher_distance = self.create_publisher(Float64, 'distance', 1)
        self.subscription = self.create_subscription(SatMsgRcv, self.gps_name + '/topic_gps',self.listener_callback, 1)
        self.subscription

    def listener_callback(self, msg_received):
        """Subscriber to roof-mounted GPS topic, display in log the solution status,
        and publish the distance between the two GPS (float type)."""

        # == Synchronize ==
        timer_start_outside = float(msg_received.timer[0:9])
        timer_start_inside = self.list_data[self.line_nbr][1][:-2]
        timer_start_inside = float(timer_start_inside.replace(':',''))
        print(timer_start_outside,timer_start_inside,)
        if timer_start_outside != timer_start_inside:
            self.get_logger().error('ERROR: Not correctly synchronized')
            diff = timer_start_outside - timer_start_inside
            print('IMPORTANT: ',diff)
            int_diff = int(round(diff*10))
            print('IMPORTANT: ',int_diff)
            if int_diff<0:
                self.solutions_dict['Lost of inside GPS'] += 1

                self.get_logger().error('GPS inside Lost')
            else:
                if int_diff > 300:  # When passing between to the next minute
                    int_diff -= 400
                self.line_nbr += int_diff
                self.solutions_dict['Lost'] += int_diff

        # == Solution status ==
        if msg_received.status == 4:
            self.solutions_dict["Fix"] += 1
        elif msg_received.status == 5:
            self.solutions_dict["Float"] += 1
        else :
            self.solutions_dict["Single"] += 1
        self.get_logger().info('Car roof GPS: "%s"' % self.solutions_dict)

        # == Distance ==
        print(msg_received.latitude, msg_received.longitude,
                         float(self.list_data[self.line_nbr][2]), float(self.list_data[self.line_nbr][3]))
        g = geod.Inverse(msg_received.latitude, msg_received.longitude,
                         float(self.list_data[self.line_nbr+self.temporal_shift][2]),
                         float(self.list_data[self.line_nbr+self.temporal_shift][3]))
        self.line_nbr += 1
        msg_send = Float64()
        msg_send.data = g['s12']
        print('distance:',g['s12'])
        self.publisher_distance.publish(msg_send)

        self.count+=1
        print('count:',self.count)
        print('=======')


def main(args=None):
    """ROS 2 node main."""
    rclpy.init(args=args)

    Gps_Sol_Status_And_Distance = GpsSolStatusAndDistance()

    rclpy.spin(Gps_Sol_Status_And_Distance)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Gps_Sol_Status_And_Distance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
