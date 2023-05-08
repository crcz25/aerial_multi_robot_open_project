"""UWB Simulator - Using a configuration file"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os
import yaml


def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('robots_description'),
        'worlds',
        'empty_world.world'
    )

    ns = 'T01'
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    model_sdf_path = os.path.join(
        get_package_share_directory('robots_description'),
        'models',
        model_folder,
        'model.sdf'
    )

    launch_description = LaunchDescription()

    # Launch Gazebo, loading simple world
    launch_gazebo_world = ExecuteProcess(
        cmd=
        [
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ],
        output='screen'
    )

    # Spaw  a turtlebot
    spawn_turtlebot = Node(
        package='robots_description',
        executable='inject_entity.py',
        output='screen',
        arguments=[
            model_sdf_path,
            '0',
            '0',
            '0',
            '0.001',
            ns
        ]
    )

    launch_description.add_action(launch_gazebo_world)
    launch_description.add_action(spawn_turtlebot)

    return launch_description
