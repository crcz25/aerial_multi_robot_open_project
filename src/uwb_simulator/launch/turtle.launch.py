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

    # Load the robots model
    TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]
    model_folder = "turtlebot3_" + TURTLEBOT3_MODEL
    tbot_sdf_path = os.path.join(
        get_package_share_directory("robots_description"),
        "models",
        model_folder,
        "model.sdf",
    )
    tbot_urdf_path = os.path.join(
        get_package_share_directory("robots_description"),
        "urdf",
        f"{model_folder}.urdf",
    )
    with open(tbot_urdf_path, "r") as tbot_robot_file:
        tbot_robot_desc = tbot_robot_file.read()

    robot_ns = "T01"
    # Publish static transforms
    tbot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_ns,
        output="screen",
        parameters=[
            {
                "robot_description": tbot_robot_desc,
                "use_sim_time": True,
                "frame_prefix": f"{robot_ns}_",
            },
        ],
        # remappings=remappings
    )
    # Spawn a turtlebot
    tbot_start_gazebo_ros_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            robot_ns,
            "-file",
            tbot_sdf_path,
            "-robot_namespace",
            robot_ns,
            "-x",
            str(3.0),
            "-y",
            str(3.0),
            "-timeout",
            "120.0",
        ],
    )
    # Rename the base_link frame
    tbot_base_rename = Node(
        package="uwb_simulator",
        executable="turtle_tf2_base_rename",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_name": robot_ns}],
    )
    launch_description.add_action(tbot_state_pub)
    launch_description.add_action(tbot_start_gazebo_ros_spawner_cmd)
    launch_description.add_action(tbot_base_rename)

    return launch_description
