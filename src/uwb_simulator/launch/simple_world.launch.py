"""UWB Simulator - Using a configuration file"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os
import yaml


def generate_launch_description():
    ns = 'T01'

    world_path = os.path.join(
        get_package_share_directory('robots_description'),
        'worlds',
        'empty_world.world'
    )
    # urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')

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

    launch_description.add_action(launch_gazebo_world)

    return launch_description
