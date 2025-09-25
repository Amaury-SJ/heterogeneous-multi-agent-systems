"""Launch file to display our system in RViz2."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    """Launch nodes : rviz2, rviz2_map_visualization and rviz2_gps_visualization."""
    display_ns = LaunchConfiguration('display_ns')
    display_ns_arg = DeclareLaunchArgument(
        'display_ns', default_value='display_default')

    return LaunchDescription([
        display_ns_arg,
        Node(
            package='rviz2',
            executable='rviz2',
            namespace=display_ns,
            #name='rviz2',
            output='screen',
            arguments=['-d', os.getcwd() + '/src/display_rviz2_pckg/rviz/config_of_rviz2.rviz']
        ),
        Node(
            package='display_rviz2_pckg',
            executable='rviz2_map_visualization',
            namespace=display_ns,
            output='screen',
            emulate_tty=True,
            parameters=[ # Choose manually the map: 'map_plaine', 'map_potager', etc
                {'name_map': 'map_loria', 'reference_agent_name': 'human_1', 'antenna_height': 0.065}
            ]
        ),
        Node(
            package='display_rviz2_pckg',
            executable='rviz2_gps_visualization',
            namespace=display_ns,
            output='screen',
            emulate_tty=True,
            parameters=[ # Only agent with Emlid RTK GPS
                {'agents_name': ['human_1',]} # {'agents_name': ['human_1', 'spot_1']}

            ]
        ),
    ])
