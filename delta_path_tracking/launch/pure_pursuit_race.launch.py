import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	use_sim_time = LaunchConfiguration("use_sim_time")
	use_rviz = LaunchConfiguration("use_rviz")
	rviz_config = LaunchConfiguration("rviz_config")

	gazebo_share = get_package_share_directory("delta_gazebo")

	pure_pursuit_node = Node(
		package="delta_path_tracking",
		executable="pure_pursuit_node",
		name="pure_pursuit_node",
		output="screen",
		parameters=[
			{
				"use_sim_time": use_sim_time,
				"path_topic": "/planned_path",
				"cmd_vel_topic": "/cmd_vel_nav",
				"use_StartFlag": True,
				"use_ttc": True,
				"control_rate_hz": 50.0,
				"goal_tolerance": 0.3,
				"v_min": 1.0,
				"v_max": 4.75,
				"adaptative_v":True,
				"pub_errs": True,
				"pub_debug": False,
				"lookahead_kv": 0.4,
				"narrow_speed_factor": 0.9
			}
		],
	)

	ttc_break_node = Node(
		package="delta_nav",
		executable="ttc_break_node",
		name="ttc_break_node",
		output="screen",
		parameters=[{
			"use_sim_time": use_sim_time, 
			"ttc_threshold": 0.5,
			"min_distance_threshold": 0.6,
			"forward_angle_range": 8.0,
			"safety_bubble": True,
			"heartbeat_rate_hz" : 1.0
			}],
	)

	rviz2_node = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="screen",
		arguments=["-d", rviz_config],
		parameters=[{"use_sim_time": use_sim_time}],
		condition=IfCondition(use_rviz),
	)

	return LaunchDescription(
		[
			DeclareLaunchArgument(
				"use_sim_time",
				default_value="true",
				description="Use simulation (Gazebo) clock if true.",
			),
			DeclareLaunchArgument(
				"use_rviz",
				default_value="true",
				description="If true launch RViz2 with the configured path-tracking view.",
			),
			DeclareLaunchArgument(
				"rviz_config",
				default_value=os.path.join(gazebo_share, "config", "rviz2_config_single_plan.rviz"),
				description="Full path to the RViz2 config file.",
			),
			pure_pursuit_node,
			ttc_break_node,
			rviz2_node,
		]
	)
