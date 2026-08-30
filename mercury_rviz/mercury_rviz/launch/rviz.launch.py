from launch.actions import GroupAction, DeclareLaunchArgument, Shutdown
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration as LC
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

import os

robot = 'mercury'

config = PathJoinSubstitution([
    get_package_share_directory('mercury_descriptions'),
    LC("robot"),
    'config',
    LC("robot_xacro_yaml")
])

rviz_pkg_dir = get_package_share_directory('mercury_rviz')

mercury_rviz_src = os.path.join(rviz_pkg_dir[: rviz_pkg_dir.find(
    "install") - 1], "src", "mercury_gui", "mercury_rviz")


def generate_launch_description():
    return LaunchDescription([
        GroupAction(
            actions=[
                # declare the launch args here
                DeclareLaunchArgument(
                    "robot",
                    default_value=robot,
                    description="Name of the vehicle",
                ),

                DeclareLaunchArgument(
                    "control_config_file",
                    default_value=["control_config_", LC("robot"), ".rviz"]
                ),

                DeclareLaunchArgument('robot_xacro_yaml', default_value=[
                                      LC("robot"), '_xacro_frames' '.yaml']),

                # start rviz
                Node(
                    package='rviz2',
                    executable='rviz2',
                    on_exit=Shutdown(),
                    arguments=[
                        "-d",
                        PathJoinSubstitution([
                            mercury_rviz_src,
                            LC("control_config_file")
                        ])
                    ],
                ),

                # send the rest into the robot namespace
                PushRosNamespace(["/", LC('robot')]),

                # start the thruster wrench visualizer
                Node(
                    package="mercury_controller",
                    executable="thruster_wrench_publisher.py",
                    name="thruster_wrench_publisher",
                    output="screen",
                    parameters=[
                        {"vehicle_config": config},
                        {"robot": LC("robot")},
                    ]
                ),

                Node(
                    package="mercury_rviz",
                    executable="MarkerPublisher.py",
                    name="marker_publisher",
                    output="screen",
                    parameters=[
                        os.path.join(get_package_share_directory(
                            "mercury_rviz"), "config", "markers.yaml")
                    ]
                )
            ]
        )
    ])
