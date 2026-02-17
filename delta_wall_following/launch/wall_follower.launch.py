import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # -- Controllers ---
    dist_finder = Node(
        package="delta_wall_following",
        executable="dist_finder",
        output="screen",
        parameters=[
            {"wall_distance": 2.5},
            {"angle_th": 50.0},
            {"max_discontinuity": 6.0},
            {"pub_logger": False},      #Flag to pub logger info
        ]
    )

    control = Node(
        package="delta_wall_following",
        executable="control",
        output="screen",
        parameters=[
            {"forward_velocity": 2.5},
            {'brake_turn_angle': 1.3},
            {"pub_logger": False},      #Flag to pub logger info
        ]
    )

    aebs = Node(
        package="delta_nav",
        executable="ttc_break_node",
        output="screen",
        parameters=[
            {"ttc_threshold": 0.7},            # seconds - TTC threshold for emergency braking
            {"min_distance_threshold": 0.5},   # meters - minimum distance to obstacle for braking
            {"forward_angle_range": 10.0},     # degrees - angle range in front of robot to consider
            {"max_range": 10.0},                # meters - maximum range of the sensor
        ]
    )

   

    return LaunchDescription([
        aebs,
        dist_finder,
        control,
    ])