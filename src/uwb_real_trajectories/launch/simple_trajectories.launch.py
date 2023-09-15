from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

import os
from pathlib import Path

def generate_launch_description():
    launch_description = LaunchDescription()

    # Rename the base_link frame
    turtlebot_one_traj = Node(
        package='uwb_real_trajectories',
        executable='tbot_trajectories_opti',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'n_loop': 1,
            'trajectory': 'square',
            'sim': False
        }],
    )

    # Rename the base_link frame
    turtlebot_two_traj = Node(
        package='uwb_real_trajectories',
        executable='tbot_trajectories_opti',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'n_loop': 1,
            'trajectory': 'square',
            'sim': False,
            'domain_bridge': True
        }],
    )

    launch_description.add_action(turtlebot_one_traj)
    launch_description.add_action(turtlebot_two_traj)

    return launch_description
