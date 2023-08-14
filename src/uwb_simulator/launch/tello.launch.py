"""UWB Simulator - Using a configuration file"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

import os
from pathlib import Path

from uwb_simulator.config_launcher import ConfigLaunch
from itertools import combinations, permutations, product


def generate_launch_description():
    launch_description = LaunchDescription()

    # To separate transforms between robots and namespaces
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    world_path = os.path.join(
        get_package_share_directory("robots_description"), "worlds", "empty_world.world"
    )
    # Launch Gazebo, loading simple world
    launch_gazebo_world = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",  # Publish /clock
            "-s",
            "libgazebo_ros_factory.so",  # Provide gazebo_ros::Node
            world_path,
        ],
        output="screen",
    )
    launch_description.add_action(launch_gazebo_world)

    tello_urdf_path = os.path.join(
        get_package_share_directory("robots_description"), "urdf", "tello.urdf"
    )
    tello_xacro_path = os.path.join(
        get_package_share_directory("robots_description"), "urdf", "tello.urdf.xacro"
    )

    with open(tello_xacro_path, "r") as tello_robot_file:
        tello_robot_desc = tello_robot_file.read()

    # Load the robots configuration
    CONFIG_SIMULATOR_PATH = (
        Path.cwd() / "src" / "uwb_simulator" / "config" / "simulation.yaml"
    )
    config_launch = ConfigLaunch(config_path=str(CONFIG_SIMULATOR_PATH))

    # Spawn the robots
    robots_names_conf, robots_pos_conf = config_launch.get_robots_from_config()
    uwb_nodes_info = config_launch.get_uwb_nodes_from_config()
    uwb_ranges_in_config = config_launch.get_uwb_ranges_from_config()
    demo_trajectory = config_launch.get_trajectory_from_config()

    # Get UWB Nodes config dictionary
    uwb_nodes_in_config = uwb_nodes_info[0]
    # Get the UWB Nodes' names (antennas)
    antennas_names = uwb_nodes_info[1]
    # Get the UWB Nodes' positions
    antennas_positions = uwb_nodes_info[2]

    robot_ns = "tello"
    # Publish static transforms
    tello_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_ns,
        output="screen",
        parameters=[
            {
                # 'robot_description': tello_robot_desc,
                "robot_description": ParameterValue(
                    Command(
                        ["xacro ", tello_xacro_path, " robot_name:=", robot_ns]
                    ),
                    value_type=str,
                ),
                "use_sim_time": True,
            },
        ],
        # remappings=remappings,
    )
    # Spawn a tello drone
    tello_start_gazebo_ros_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            robot_ns,
            "-robot_namespace",
            robot_ns,
            "-x",
            str(0.0),
            "-y",
            str(0.0),
            "-topic",
            f"{robot_ns}/robot_description",
            "-timeout",
            "120.0",
        ],
    )
    # Joystick driver, generates /namespace/joy messages
    tello_joy = Node(
        package="joy",
        executable="joy_node",
        output="screen",
        namespace=robot_ns,
    )
    # Joystick controller, generates /namespace/cmd_vel messages
    tello_controller = Node(
        package="tello_driver",
        executable="tello_joy_main",
        output="screen",
        namespace=robot_ns,
    )
    launch_description.add_action(tello_state_pub)
    launch_description.add_action(tello_joy)
    launch_description.add_action(tello_controller)
    launch_description.add_action(tello_start_gazebo_ros_spawner_cmd)

    # Rename the base_link frame
    drone_base_rename = Node(
        package="uwb_simulator",
        executable="drone_tf2_base_rename",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_name": robot_ns}],
    )

    launch_description.add_action(drone_base_rename)

    return launch_description
