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
    pkg_name = 'real_leo'

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

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

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behaviour_server',
        'bt_navigator',
        'smoother_server',
    ]

    # LOAD PARAMETERS FROM YAML FILES
    config_bt_nav     = PathJoinSubstitution([get_package_share_directory(pkg_name), 'config', 'bt_nav.yaml'])
    config_planner    = PathJoinSubstitution([get_package_share_directory(pkg_name), 'config', 'planner.yaml'])
    config_controller = PathJoinSubstitution([get_package_share_directory(pkg_name), 'config', 'controller.yaml'])

    # Behaviour Tree Navigator
    node_bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        # parameters=[config_bt_nav,{'default_nav_to_pose_bt_xml' : bt_xml_navtopose_file}],
        parameters=[config_bt_nav],
        remappings=remappings,
    )

    # Behaviour Tree Server
    node_behaviour = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behaviour_server',
        output='screen',
        parameters=[config_bt_nav],
        remappings=remappings,
    )

    # Planner Server Node
    node_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[config_planner],
        remappings=remappings,
    )

    # Controller Server Node
    node_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[config_controller],
        remappings=remappings,
    )

    node_smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[PathJoinSubstitution([get_package_share_directory(pkg_name), 'config', 'simple_smoother.yaml'])],
    )

    # Lifecycle Node Manager to automatically start lifecycles nodes (from list)
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
    )

    node_explore = Node(
        package="explore_lite",
        name="explore",
        executable="explore",
        parameters=[PathJoinSubstitution([get_package_share_directory(pkg_name), 'config', 'params.yaml']), {"use_sim_time": True}],
        output="screen",
        remappings=remappings,
    )

    ld.add_action(launch_slamtoolbox)
    # ld.add_action(launch_gmapping)
    ld.add_action(launch_map_saver)
    # ld.add_action(launch_key_teleop)
    ld.add_action(node_bt_nav)
    ld.add_action(node_behaviour)
    ld.add_action(node_planner)
    ld.add_action(node_controller)
    ld.add_action(node_smoother_server)
    ld.add_action(node_explore)
    ld.add_action(node_lifecycle_manager)

    return ld