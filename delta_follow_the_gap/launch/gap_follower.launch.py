import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # -- Controllers ---
    angular_error_computer = Node(
        package="delta_follow_the_gap",
        executable="ttc_gap_logger_node",
        output="screen",
        parameters=[
            {"pub_logger": False},      #Flag to pub logger info
        ]
    )

    control = Node(
        package="delta_follow_the_gap",
        executable="control_gap_ttc",
        output="screen"

    )

    aebs = Node(
        package="delta_follow_the_gap",
        executable="ttc_break_gap_node",
        output="screen",
    )

   

    return LaunchDescription([
        aebs,
        angular_error_computer,
        control,
    ])