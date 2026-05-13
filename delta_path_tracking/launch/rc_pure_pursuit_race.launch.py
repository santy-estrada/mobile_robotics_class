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

	pure_pursuit_params = {
		"use_sim_time": use_sim_time,
		"path_topic": "/planned_path",
		"cmd_vel_topic": "/cmd_vel_nav",
		"use_StartFlag": True,
		"use_ttc": True,
		"control_rate_hz": 20.0, #10 original
		"goal_tolerance": 0.3,
		"max_cmd_velocity": 2.80,
		"speed_adaptive_min": 2.80,
		"speed_heading_coupling": 0.5, # 0.5 original
		"cmd_smoothing_factor": 0.2, # 0.3 original
		"error_threshold": 10.0, # 10.0 original
		"use_speed_adaptive": True,
		"speed_nominal": 2.6,
		"pub_errs": True,
		"pub_debug": False,
		"lookahead_L0": 0.9, # 0.9 original
		"lookahead_kv": 0.7, # 0.7 original
		"lookahead_min": 1.0, # 1.0 original
		"lookahead_max": 2.7	, # 2.0 original
	}

	ttc_break_params = {
		"use_sim_time": use_sim_time,
		"ttc_threshold": 0.5,
		"min_distance_threshold": 0.4,
		"forward_angle_range": 20.0,
		"safety_bubble": True,
		"heartbeat_rate_hz": 1.0,
	}

	rc_pure_pursuit_node = Node(
		package="delta_path_tracking",
		executable="rc_pure_pursuit_node",
		name="rc_pure_pursuit_node",
		output="screen",
		parameters=[pure_pursuit_params],
	)

	rc_ttc_brake_node = Node(
		package="delta_nav",
		executable="rc_ttc_brake_node",
		name="rc_ttc_brake_node",
		output="screen",
		parameters=[ttc_break_params],
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
				default_value="false",
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
			rc_pure_pursuit_node,
			rc_ttc_brake_node,
			rviz2_node,
		]
	)
