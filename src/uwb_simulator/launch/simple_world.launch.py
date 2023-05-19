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

    # To separate transforms between robots and namespaces
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

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

    # Load the robots model
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    tbot_sdf_path = os.path.join(
        get_package_share_directory('robots_description'),
        'models',
        model_folder,
        'model.sdf'
    )
    tbot_urdf_path = os.path.join(
        get_package_share_directory('robots_description'),
        'urdf',
        f'{model_folder}.urdf'
    )
    with open(tbot_urdf_path, 'r') as tbot_robot_file:
        tbot_robot_desc = tbot_robot_file.read()

    tello_urdf_path = os.path.join(
        get_package_share_directory('robots_description'),
        'urdf',
        'tello.urdf'
    )
    with open(tello_urdf_path, 'r') as tello_robot_file:
        tello_robot_desc = tello_robot_file.read()

    # Load the robots configuration
    CONFIG_SIMULATOR_PATH = (
        Path.cwd() / 'src' / 'uwb_simulator' / 'config' / 'simulation.yaml'
    )
    config_launch = ConfigLaunch(
        config_path=str(CONFIG_SIMULATOR_PATH)
    )

    # Spawn the robots
    robots_in_config = config_launch.get_robots_from_config()
    uwb_nodes_in_config = config_launch.get_uwb_nodes_from_config()
    print("YAML FILE")
    print(uwb_nodes_in_config)

    for num, robot in enumerate(robots_in_config):
        print(num, robot)
        if "tello" in robot.lower():
            robot_ns = robot
            # Publish static transforms
            tello_state_pub = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                # namespace=robot_ns,
                output='screen',
                parameters=[
                    {
                        'robot_description': tello_robot_desc,
                        'use_sim_time': True,
                    },
                ],
                # remappings=remappings,
            )
            # Spawn a tello drone
            tello_start_gazebo_ros_spawner_cmd = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                output='screen',
                arguments=[
                    '-entity', robot_ns,
                    '-file', tello_urdf_path,
                    '-robot_namespace', robot_ns,
                    '-x', str(num + 2), '-y', str(num + 2),
                    '-timeout', '120.0'
                ]
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
            launch_description.add_action(tello_state_pub)
            launch_description.add_action(tello_joy)
            launch_description.add_action(tello_controller)
            launch_description.add_action(tello_start_gazebo_ros_spawner_cmd)
        else:
            robot_ns = robot
            # Publish static transforms
            tbot_state_pub = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=robot_ns,
                output='screen',
                parameters=[
                    {
                        'robot_description': tbot_robot_desc,
                        'use_sim_time': True,
                    },
                ],
                remappings=remappings
            )
            # Spawn a turtlebot
            tbot_start_gazebo_ros_spawner_cmd = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                output='screen',
                arguments=[
                    '-entity', robot_ns,
                    '-file', tbot_sdf_path,
                    '-robot_namespace', robot_ns,
                    '-x', str(num + 2), '-y', str(num + 2),
                    '-timeout', '120.0'
                ]
            )
            # Add transforms between the robot and the antennas
            tbot_transforms = Node(
                package='uwb_simulator',
                executable='antenna_tf_broadcaster',
                output='screen',
                emulate_tty=True,
                arguments=[
                    str(robot), # robot_name
                    str(uwb_nodes_in_config[robot]['num_antennas']), # num_antennas
                    uwb_nodes_in_config[robot]['names'], # names_antennas
                ]
            )
            
            launch_description.add_action(tbot_state_pub)
            launch_description.add_action(tbot_start_gazebo_ros_spawner_cmd)
            launch_description.add_action(tbot_transforms)

    return launch_description
