import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import xacro

def generate_launch_description():
    ld = LaunchDescription()
    pkg_name = 'my_leo'

    # LOAD PARAMETERS FROM YAML FILES
    config_bt_nav = PathJoinSubstitution([get_package_share_directory(pkg_name), 'config', 'bt_nav.yaml'])

    # Include SLAM Toolbox standard launch file
    launch_slamtoolbox = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch', '/online_async_launch.py']),
    launch_arguments={}.items(),
    )

    launch_map_saver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('nav2_map_server'), '/launch', '/map_saver_server.launch.py']),
    launch_arguments={}.items(),
    )

    launch_key_teleop = Node(
        name="key_teleop_node",
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        output='screen',
        prefix=["xterm -e"],
        parameters=[
            {'speed': '0.4'},
            {'turn': '1.0'},
            {'repeat_rate': '10.0'},
            {'key_timeout': '0.3'},],
        remappings=[("/cmd_vel", "cmd_vel")],
    )

    launch_node_joy_linux = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        parameters=[
            {"dev": "/dev/input/js0"},
            {"deadzone": "0.1"},
            {"coalesce_interval": "0.05"},
            {"autorepeat_rate": "10.0"},
        ],
    )

    launch_node_joy_teleop = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="joy_teleop_node",
        remappings=[("/cmd_vel", "cmd_vel")],
        parameters=[config_bt_nav],
    )



    ld.add_action(launch_slamtoolbox)
    ld.add_action(launch_map_saver)
    ld.add_action(launch_key_teleop)
    # ld.add_action(launch_node_joy_linux)
    # ld.add_action(launch_node_joy_teleop)

    return ld