from launch import LaunchDescription
from launch_ros.actions import Node,SetRemap
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers.on_shutdown import OnShutdown
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events import matches_node_name
from launch.actions import LogInfo
import lifecycle_msgs.msg
import os

def generate_launch_description():

    declared_arguments = []


    log_level = LaunchConfiguration('log_level')

    declared_arguments.append(
        DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level'))

    config = os.path.join(
        get_package_share_directory('nav_follow'),
        'config',
        'pid.yaml'
        )

    node1 = [Node(
                package='nav_follow',
                executable='nav_follow_node',
                namespace="sweepee_2",
                output='screen',
                # prefix=['xterm -e gdb -ex run --args'],
                # prefix=['valgrind --tool=memcheck --leak-check=yes --show-reachable=yes '],
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{"laser_scan_slave": "/sweepee_2/front_laser_plugin/out"},
                            {"laser_scan_master": "/sweepee_1/front_laser_plugin/out"},
                            {"cmd_vel_topic": "/sweepee_2/cmd_vel"},
                            {"frame_name_laser_slave":  "sweepee_2/front_laser"},
                            {"frame_name_laser_master": "sweepee_1/front_laser"},
                            {"frame_name_base_slave":   "sweepee_2/base_footprint"},
                            {"frame_name_base_master":  "sweepee_1/base_footprint"},
                            {"icp_iterations"                     : 15},
                            {"icp_TransformationEpsilon"          : 1e-9},
                            {"icp_EuclideanFitnessEpsilon"        : 1.0},
                            {"icp_RANSACOutlierRejectionThreshold": 1.5},
                            {"icp_MaxCorrespondenceDistance"      : 100.0},
                            {"use_sim_time"                       : True},                
                            {"enable_tf"                          : True},
                            {"enable_vel_feedforward"             : True},
                            {"enable_icp"                         : False}, 
                            {"cmd_vel_topic_master"               : "/sweepee_1/cmd_vel"},

                            config])]

    foll_manager = [Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace="sweepee_2",
            name='lifecycle_manager_follow',
            output='screen',
            arguments=['--ros-args', '--log-level', "info"],
            parameters=[{'autostart': True},
                        {'node_names': ['nav_follow']},
                        {'bond_timeout': 0.0},
                        {"use_sim_time": True}])]

    shutdown_event = [RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                  lifecycle_node_matcher=matches_node_name(node_name="sweepee_2/node_name"),
                  transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is exiting."),
            ],
        )
    )]

    # node2 = [Node(
    #             package='nav_follow',
    #             executable='nav_follow_node',
    #             namespace="sweepee_3",
    #             output='screen',
    #             # prefix=['xterm -e gdb -ex run --args'],
    #             # prefix=['valgrind --tool=memcheck --leak-check=yes --show-reachable=yes '],
    #             arguments=['--ros-args', '--log-level', log_level],
    #             parameters=[{"laser_scan_slave": "/sweepee_3/front_laser_plugin/out"},
    #                         {"laser_scan_master": "/sweepee_1/front_laser_plugin/out"},
    #                         {"cmd_vel_topic": "/sweepee_3/cmd_vel"},
    #                         {"frame_name_laser_slave":  "sweepee_3/front_laser"},
    #                         {"frame_name_laser_master": "sweepee_1/front_laser"},
    #                         {"frame_name_base_slave":   "sweepee_3/base_footprint"},
    #                         {"frame_name_base_master":  "sweepee_1/base_footprint"},
    #                         {"icp_iterations"                     : 15},
    #                         {"icp_TransformationEpsilon"          : 1e-9},
    #                         {"icp_EuclideanFitnessEpsilon"        : 1.0},
    #                         {"icp_RANSACOutlierRejectionThreshold": 1.5},
    #                         {"icp_MaxCorrespondenceDistance"      : 100.0},
    #                         {"use_sim_time"                       : True},                
    #                         {"enable_tf"                          : True},
    #                         {"enable_vel_feedforward"             : True},
    #                         {"enable_icp"                         : False}, 
    #                         {"cmd_vel_topic_master"               : "/sweepee_1/cmd_vel"},
    #                         {"autostart"                          : True},

    #                         config])]

    return LaunchDescription(declared_arguments + node1 + foll_manager + shutdown_event)
