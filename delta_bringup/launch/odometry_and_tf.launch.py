import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # =========================================================
    # Launch existentes
    # =========================================================

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            "/home/santy-estrada/mrad_ws_2601_delta/src/delta_bringup/launch/rsp.launch.py"
        )
    )

    rc_joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            "/home/santy-estrada/mrad_ws_2601_delta/src/delta_bringup/launch/rc_car_joy.launch.py"
        )
    )

    # =========================================================
    # Nodo Madgwick
    # =========================================================

    madgwick_node = Node(
        package="delta_ekf",
        executable="madgwick_filter_node",
        name="madgwick_filter_node",
        output="screen",
    )

    # =========================================================
    # Nodo TF
    # =========================================================

    tf_node = Node(
        package="delta_ekf",
        executable="tf_ph_nostamp_node",
        name="tf_ph_nostamp_node",
        output="screen",
    )

    # =========================================================
    # Nodo Encoder
    # =========================================================

    encoder_node = Node(
        package="delta_ekf",
        executable="encoder_subs_node",
        name="encoder_subs_node",
        output="screen",
    )

    # =========================================================
    # Return launch description
    # =========================================================

    return LaunchDescription([
        rsp_launch,
        madgwick_node,
        tf_node,
        encoder_node,
        rc_joystick,
    ])