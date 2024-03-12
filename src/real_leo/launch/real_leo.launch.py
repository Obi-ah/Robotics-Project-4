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

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'real_leo'

    robot_desc = xacro.process_file(
        os.path.join(
            get_package_share_directory(pkg_name),
            "urdf",
            "leo_sim.urdf.xacro",
        ),
    ).toxml()

    # Launch robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output='screen',
        parameters=[
            {"robot_description": robot_desc},
        ],
    )

    imu_filter = Node(
      package="imu_filter_madgwick",
      executable="imu_filter_madgwick_node",
      name="imu_filter_node",
      parameters=[PathJoinSubstitution([get_package_share_directory(pkg_name),'config','imu_filter_node.yaml'])],
    )

    ekf_localization = Node(
       package="robot_localization",
       executable="ekf_node",
       name="ekf_node",
       parameters=[PathJoinSubstitution([get_package_share_directory(pkg_name),'config','ekf_node.yaml'])],
    )

    # Rviz node
    node_rviz = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory(pkg_name), '/launch', '/rviz_leo.launch.py']),
    launch_arguments={}.items(),
    )

    navigation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory(pkg_name), '/launch', '/navigation.launch.py']),
    launch_arguments={}.items(),
    )

    joy_stick = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory(pkg_name), '/launch', '/teleop_joy.launch.py']),
    launch_arguments={}.items(),
    )

    # Add actions to LaunchDescription
    # ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(robot_state_publisher)
    ld.add_action(imu_filter)
    ld.add_action(ekf_localization)
    ld.add_action(node_rviz)    
    ld.add_action(navigation)
    # ld.add_action(joy_stick)
    
    return ld
