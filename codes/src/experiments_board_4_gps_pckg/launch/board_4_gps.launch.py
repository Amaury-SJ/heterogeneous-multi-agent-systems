from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    return LaunchDescription([
        
        # ################### Display in 3D in RViz 2 #################
        Node(
            package='rviz2',
            executable='rviz2',

            name='rviz2',
            output='screen',
            arguments=['-d', os.getcwd() + '/src/display_rviz2_pckg/rviz/rviz2_for_board_4_gps.rviz']
        ),
        Node(
            package='display_rviz2_pckg',
            executable='rviz2_map_visualization',

            output='screen',
            emulate_tty=True,
            parameters=[
                {'name_map': 'map_plaine', 'reference_agent_name': 'bottom_left'}
            ]
        ),
        Node(
            package='display_rviz2_pckg',
            executable='rviz2_gps_visualization',

            output='screen',
            emulate_tty=True,
            parameters=[
                {'agents_name': ['top_left', 'top_right', 'bottom_left', 'bottom_right']}
            ]
        ),
        
        # ################ Calculate distances #####################
        Node(
            package='experiments_board_4_gps_pckg',
            executable='calculate_distance_gps',
            name='tl_tr',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'first_gps': 'top_left', 'second_gps': 'top_right'}
            ]
        ),
        Node(
            package='experiments_board_4_gps_pckg',
            executable='calculate_distance_gps',
            name='tr_br',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'first_gps': 'top_right', 'second_gps': 'bottom_right'}
            ]
        ),
        Node(
            package='experiments_board_4_gps_pckg',
            executable='calculate_distance_gps',
            name='br_bl',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'first_gps': 'bottom_right', 'second_gps': 'bottom_left'}
            ]
        ),
        Node(
            package='experiments_board_4_gps_pckg',
            executable='calculate_distance_gps',
            name='bl_tl',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'first_gps': 'bottom_left', 'second_gps': 'top_left'}
            ]
        ),
        
        # ################## Recuperate data from GNSS via wifi ##################
        # Node(
        #     package='gps_rtk_pckg',
        #     executable='gps_talker',
        #     namespace='top_left',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[
        #         {'ip_host': '10.42.0.112'}
        #     ]
        # ),
        # Node(
        #     package='gps_rtk_pckg',
        #     executable='gps_talker',
        #     namespace='top_right',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[
        #         {'ip_host': '10.42.0.18'}
        #     ]
        # ),
        # Node(
        #     package='gps_rtk_pckg',
        #     executable='gps_talker',
        #     namespace='bottom_left',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[
        #         {'ip_host': '10.42.0.82'}
        #     ]
        # ),
        # Node(
        #     package='gps_rtk_pckg',
        #     executable='gps_talker',
        #     namespace='bottom_right',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[
        #         {'ip_host': '10.42.0.143'}
        #     ]
        # ),

    ])
