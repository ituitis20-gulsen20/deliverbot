#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
	worldFileName = 'NMaze-1.world'
	map_file = "my_slam_map_newMaze_alt1.yaml"
	use_sim_time = LaunchConfiguration('use_sim_time', default='True')

	pkg_share_dir = get_package_share_directory('project')
	pkg_prefix_dir = get_package_prefix('project')
	pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
	pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

	world_file_path = os.path.join(pkg_share_dir, 'worlds', worldFileName)
	rviz_config_path = os.path.join(pkg_share_dir, 'rviz', 'nav.rviz')
	map_path = os.path.join(pkg_share_dir, 'maps', map_file)

	gzserver_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
		),
		launch_arguments={'world': world_file_path}.items()
	)

	gzclient_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
		)
	)

	robot_state_publisher = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
		),
		launch_arguments={'use_sim_time': use_sim_time}.items()
	)

	cartographer = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_share_dir, 'launch', 'cartographer.launch.py')
		),
		launch_arguments={'use_sim_time': use_sim_time}.items()
	)

	nav2 = Node(package='nav2_map_server',
				executable='map_server',
				name='map_server',

				output='screen',
				parameters=[{'use_sim_time': use_sim_time},
							{'yaml_filename': map_path},
							{'topic_name': 'map'},
							{'frame_id': 'map'},
							])
	amcl = Node(
		package='nav2_amcl',
		executable='amcl',
		name='amcl',
		output='screen',
		parameters=[{'initial_pose': {'x': 5.5, 'y': -0.5, 'z': 0.0, 'yaw': 3.15}},
					{'set_initial_pose': True},
					{'first_map_only': True},
					{'use_sim_time': use_sim_time}])
	lifecycle = Node(package='nav2_lifecycle_manager',
					 executable='lifecycle_manager',
					 name='lifecycle_manager_mapper',
					 output='screen',
					 parameters=[{'use_sim_time': use_sim_time},
								 {'autostart': True},
								 {'node_names': ['amcl', 'map_server']}])


	referee_node = Node(
		package='project',
		executable='cargo_tracker.py',
		name='tracker',
		output='screen',
	)

	rviz = Node(
		package='rviz2',
		node_executable='rviz2',
		node_name='rviz2',
		arguments=['-d', rviz_config_path,
				   '--ros-args', '--remap', 'use_sim_time:=true'],
		parameters=[{'use_sim_time': use_sim_time}],
		output='log')

	return [gzserver_cmd, gzclient_cmd, robot_state_publisher, rviz, referee_node, cartographer]

def generate_launch_description():
	# Argumments Declaration

	# Launch Description Declaration
	ld = LaunchDescription()

	# Add Actions

	ld.add_action(OpaqueFunction(function=launch_setup))

	return ld