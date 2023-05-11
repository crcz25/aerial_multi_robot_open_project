"""UWB Simulator - Using a configuration file"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os
from pathlib import Path

from uwb_simulator.config_launcher import ConfigLaunch


def generate_launch_description():
    launch_description = LaunchDescription()

    world_path = os.path.join(
        get_package_share_directory('robots_description'),
        'worlds',
        'empty_world.world'
    )
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

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    model_sdf_path = os.path.join(
        get_package_share_directory('robots_description'),
        'models',
        model_folder,
        'model.sdf'
    )
    tello_urdf_path = os.path.join(
        get_package_share_directory('robots_description'),
        'urdf',
        'tello.urdf'
    )

    CONFIG_SIMULATOR_PATH = (
        Path.cwd() / 'src' / 'uwb_simulator' / 'config' / 'simulation.yaml'
    )
    config_launch = ConfigLaunch(
        config_path=str(CONFIG_SIMULATOR_PATH)
    )

    robots_in_config = config_launch.get_robots_from_config()
    for num, robot in enumerate(robots_in_config):
        print(num, robot)
        if "tello" in robot.lower():
            robot_ns = robot
            # Spawn a tello drone
            spawn_tello = Node(
                package='robots_description',
                executable='inject_entity.py',
                output='screen',
                arguments=[
                    tello_urdf_path,
                    str(num + 2),
                    str(num + 2),
                    '0',
                    '0.001',
                    robot_ns,
                    robot_ns
                ],
            )
            # Publish static transforms
            tello_state_pub = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                arguments=[tello_urdf_path]
            )
            # Joystick driver, generates /namespace/joy messages
            tello_joy = Node(
                package='joy',
                executable='joy_node',
                output='screen',
                namespace=robot_ns
            )
            # Joystick controller, generates /namespace/cmd_vel messages
            tello_controller = Node(
                package='tello_driver',
                executable='tello_joy_main',
                output='screen',
                namespace=robot_ns
            )

            launch_description.add_action(spawn_tello)
            launch_description.add_action(tello_state_pub)
            launch_description.add_action(tello_joy)
            launch_description.add_action(tello_controller)
        else:
            robot_ns = robot
            # Spawn a turtlebot
            spawn_turtlebot = Node(
                package='robots_description',
                executable='inject_entity.py',
                output='screen',
                arguments=[
                    model_sdf_path,
                    str(num + 2),
                    str(num + 2),
                    '0',
                    '0.001',
                    robot_ns,
                    robot_ns
                ],
            )
            launch_description.add_action(spawn_turtlebot)

    # ns = 'T01'
    # drone_ns = 'Tello01'
    """
    # Spawn a turtlebot
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
            ns,
            ns
        ],
    )

    # Spawn a tello drone
    spawn_tello = Node(
        package='robots_description',
        executable='inject_entity.py',
        output='screen',
        arguments=[
            tello_urdf_path,
            '4',
            '4',
            '0',
            '0.001',
            drone_ns,
            drone_ns
        ],
    )
    # Publish static transforms
    tello_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=[tello_urdf_path]
    )
    # Joystick driver, generates /namespace/joy messages
    tello_joy = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        namespace=drone_ns
    )
    # Joystick controller, generates /namespace/cmd_vel messages
    tello_controller = Node(
        package='tello_driver',
        executable='tello_joy_main',
        output='screen',
        namespace=drone_ns
    )
    """

    return launch_description
