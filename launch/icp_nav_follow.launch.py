from launch import LaunchDescription
from launch_ros.actions import Node,SetRemap
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    declared_arguments = []


    log_level = LaunchConfiguration('log_level')

    declared_arguments.append(
        DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level'))

    config = os.path.join(
        get_package_share_directory('icp_nav_follow'),
        'config',
        'pid.yaml'
        )

    node1 = [Node(
                package='icp_nav_follow',
                executable='icp_nav_follow_node',
                output='screen',
                # prefix=['xterm -e gdb -ex run --args'],
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{"laser_scan_slave": "/azrael/scan"},
                            {"laser_scan_master": "/omron/scan"},
                            {"frame_name": "azrael/laser"},
                            {"cmd_vel_topic": "/azrael/cmd_vel"},
                            {"slave_frame":  "azrael/base_footprint"},
                            {"master_frame": "omron/base_link"},
                            config])]

    return LaunchDescription(declared_arguments + node1)

  