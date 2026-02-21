import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # -- Controllers ---
    error_computer = Node(
        package="delta_follow_the_gap",
        executable="gap_distance_node",
        output="screen",
        parameters=[
            {"pub_logger": False},      #Flag to pub logger info
            {"circle_radius": 0.1},
            {"max_linear_vel": 1.6},
            {"brake_turn_angle": 1.5},
            {"min_distance": 0.3},
            {"max_distance": 12.0},
            {"robot_radius": 0.1},
            {"width_weight": 1.5},
            {"depth_weight": 1.0},
        ]
    )

    control = Node(
        package="delta_follow_the_gap",
        executable="gap_distance_controller",
        output="screen",
        parameters=[
            {"kp": 1.0},
            {"forward_vel": 1.4},
            {"brake_turn_angle": 0.9},
            {"start_flag": False},
            {"pub_logger": False},
        ]

    )

    aebs = Node(
        package="delta_nav",
        executable="ttc_break_node",
        output="screen",
        parameters=[
            {"tc_threshold": 0.85},
            {"min_distance_threshold": 1.2},
            {"forward_angle_range": 18.0},
            {"rear_angle_range": 10.0},
            {"max_range": 2.0},
            {"min_range": 0.1},
            {"publish_rate": 100.0},
        ]
    )


    return LaunchDescription([
        aebs,
        error_computer,
        control,
    ])