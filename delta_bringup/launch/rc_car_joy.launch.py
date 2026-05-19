import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    bringup_pkg_name = "delta_bringup"
    
    joy_params = os.path.join(get_package_share_directory(bringup_pkg_name),'config','joystick.yaml')


    joy_node = Node(package='joy', 
                    executable='joy_node',
                    parameters=[joy_params],
    )
    teleop_node = Node(package='teleop_twist_joy', 
                    executable='teleop_node',
                    name="teleop_node",
                    parameters=[joy_params],
                    remappings=[('/cmd_vel','/cmd_vel_joy')]
    )

    twist_mux_params = os.path.join(get_package_share_directory(bringup_pkg_name),'config','twist_mux.yaml')

    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params],
                    remappings=[('/cmd_vel_out','/cmd_vel')]
    )




    return LaunchDescription([
        joy_node,
        teleop_node,
        twist_mux_node,

    ])