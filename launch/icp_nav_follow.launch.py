from launch import LaunchDescription
from launch_ros.actions import Node,SetRemap
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    declared_arguments = []


    log_level = LaunchConfiguration('log_level')

    declared_arguments.append(
        DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level'))

    node1 = [Node(
                package='icp_nav_follow',
                executable='icp_nav_follow_node',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{"laser_scan_topic": "/azrael/scan"},
                            {"frame_name": "azrael/laser"},
                            {"cmd_vel_topic": "/azrael/cmd_vel"},
                            {"slave_frame":  "azrael/base_footprint"},
                            {"master_frame": "omron/base_link"},
                            {"noss": "asd"}])]

    return LaunchDescription(declared_arguments + node1)

  