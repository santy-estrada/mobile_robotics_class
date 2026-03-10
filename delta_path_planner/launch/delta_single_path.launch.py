import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

use_waypoints = False  # Set to True to enable waypoints mode in planners


def generate_launch_description():
	use_sim_time = LaunchConfiguration("use_sim_time")
	use_amcl = LaunchConfiguration("use_amcl")

	bringup_share = get_package_share_directory("delta_bringup")
	ekf_share = get_package_share_directory("delta_ekf")

	gz_spawn_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(bringup_share, "launch", "gz_spawn.launch.py")
		),
		launch_arguments={"use_sim_time": use_sim_time, "gz_mode": "False"}.items(),
	)

	ekf_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(ekf_share, "launch", "ekf.launch.py")),
		launch_arguments={"use_sim_time": use_sim_time}.items(),
	)

	dijkstra_node = Node(
		package="delta_path_planner",
		executable="dijkstra_node",
		name="dijkstra_node",
		output="screen",
		parameters=[
			{"use_sim_time": use_sim_time,
	         "waypoints": use_waypoints,  # Set to True to enable waypoints mode
			 }],
	)

	best_first_node = Node(
		package="delta_path_planner",
		executable="best_first_node",
		name="best_first_node",
		output="screen",
		parameters=[{"use_sim_time": use_sim_time,
                    "waypoints": use_waypoints,  # Set to True to enable waypoints mode
            }],
	)


	ara_node = Node(
		package="delta_path_planner",
		executable="ara_node",
		name="ara_node",
		output="screen",
		parameters=[{"use_sim_time": use_sim_time,
                    "waypoints": use_waypoints,  # Set to True to enable waypoints mode
            }],
	)

	amcl_localization_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(bringup_share, "launch", "amcl_localization.launch.py")
		),
		launch_arguments={"use_sim_time": use_sim_time}.items(),
		condition=IfCondition(use_amcl),
	)

	slam_localization_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(bringup_share, "launch", "slam_localization.launch.py")
		),
		launch_arguments={"use_sim_time": use_sim_time}.items(),
		condition=UnlessCondition(use_amcl),
	)

	# Stagger startup so Gazebo and robot interfaces are available first.
	ekf_after_spawn = TimerAction(period=2.0, actions=[ekf_launch])
	planners_after_ekf = TimerAction(period=3.0, actions=[dijkstra_node, best_first_node, ara_node])
	localization_after_planners = TimerAction(
		period=4.0,
		actions=[amcl_localization_launch, slam_localization_launch],
	)

	return LaunchDescription(
		[
			DeclareLaunchArgument(
				"use_sim_time",
				default_value="true",
				description="Use simulation (Gazebo) clock if true.",
			),
			DeclareLaunchArgument(
				"use_amcl",
				default_value="false",
				description="If true launch AMCL; otherwise launch SLAM localization.",
			),
			gz_spawn_launch,
			ekf_after_spawn,
			planners_after_ekf,
			localization_after_planners,
		]
	)
