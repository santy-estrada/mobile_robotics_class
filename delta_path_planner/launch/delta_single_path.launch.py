import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	use_sim_time = LaunchConfiguration("use_sim_time")
	use_amcl = LaunchConfiguration("use_amcl")
	use_waypoints = LaunchConfiguration("use_waypoints")
	planner_index = LaunchConfiguration("planner_index")

	bringup_share = get_package_share_directory("delta_bringup")
	ekf_share = get_package_share_directory("delta_ekf")

	# (index, node_name, executable)
	planners = [
		(0, "dijkstra_node", "dijkstra_node"),
		(1, "best_first_node", "best_first_node"),
		(2, "ara_node", "ara_node"),
	]

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

	planner_nodes = []
	for idx, node_name, executable in planners:
		planner_nodes.append(
			Node(
				package="delta_path_planner",
				executable=executable,
				name=node_name,
				output="screen",
				parameters=[
					{
						"use_sim_time": use_sim_time,
						"waypoints": use_waypoints,
						"topics.path_topic": "/planned_path",
					}
				],
				condition=IfCondition(EqualsSubstitution(planner_index, str(idx))),
			)
		)

	waypoints_node = Node(
		package="delta_path_planner",
		executable="waypoints_node",
		name="waypoints_node",
		output="screen",
		parameters=[{"use_sim_time": use_sim_time,
			   "use_start": True,
			   "manual": True,
			   "closed_loop": True}],
		condition=IfCondition(use_waypoints),
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
	planners_after_ekf = TimerAction(period=3.0, actions=planner_nodes)
	waypoints_after_ekf = TimerAction(period=3.0, actions=[waypoints_node])
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
			DeclareLaunchArgument(
				"planner_index",
				default_value="2",
				description="Planner index: 0=dijkstra, 1=best_first, 2=ara.",
			),
			DeclareLaunchArgument(
				"use_waypoints",
				default_value="true",
				description="If true planner listens to /waypoints_topic and launches waypoints_node.",
			),
			gz_spawn_launch,
			ekf_after_spawn,
			planners_after_ekf,
			waypoints_after_ekf,
			localization_after_planners,
		]
	)
