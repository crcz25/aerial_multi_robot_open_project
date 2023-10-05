#! /usr/bin/python
import argparse
import csv
import sys
from itertools import combinations, permutations, product
from pathlib import Path
from random import randint
from time import time

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Float32
from tf_transformations import euler_from_quaternion

from tello_msgs.srv import TelloAction


class SaveData(Node):
    def __init__(self):
        # Generate a node with a random name number
        super().__init__(f"Save_Data{randint(0, 1000)}")
        self.qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Declare the parameters
        self.declare_parameter("sim", rclpy.Parameter.Type.BOOL)
        self.declare_parameter("bag", rclpy.Parameter.Type.STRING)
        self.sim = self.get_parameter_or("sim", None)
        self.bag = self.get_parameter_or("bag", None)

        if self.bag.value:
            path = Path(self.bag.value)
            # Get file ending with .yaml to get the topics in the bag
            for file in path.iterdir():
                if file.suffix == ".yaml":
                    path = file
                    break
            # Check if the file exists
            if not path.exists():
                self.get_logger().error("File does not exist")
                sys.exit(1)
            # Check if the file is a file
            if not path.is_file():
                self.get_logger().error("Path is not a file")
                sys.exit(1)
            # Open the yaml file
            with open(path, "r") as file:
                try:
                    # Load the yaml file
                    data = yaml.safe_load(file)
                    # Get all the keys topic_metadata.name to create a list of topics
                    metadata = data["rosbag2_bagfile_information"][
                        "topics_with_message_count"
                    ]
                    self.topics = [
                        topic["topic_metadata"]["name"] for topic in metadata
                    ]
                    # Filter only the topics that we want(those that contain odom, and vrpn_client_node)
                    self.topics_odom = [
                        topic for topic in self.topics if "odom" in topic
                    ]
                    self.topics_opti = [
                        topic for topic in self.topics if "vrpn_client_node" in topic
                    ]
                    print(f"Topics odom: {self.topics_odom}")
                    print(f"Topics opti: {self.topics_opti}")

                except yaml.YAMLError as error:
                    self.get_logger().error(error)
                    sys.exit(1)

        # Check if the simulation is true or false
        if self.sim.value:
            # Load the topics from the simulation
            CONFIG_SIMULATOR_PATH = (
                Path.cwd() / "src" / "uwb_simulator" / "config" / "simulation.yaml"
            )
            print(f"Config path: {CONFIG_SIMULATOR_PATH}")
            with open(CONFIG_SIMULATOR_PATH, "r") as file:
                config_dict = yaml.safe_load(file)
            # Get the topics from the config file
            uwb_nodes = config_dict.get("uwb_nodes", None)
            uwb_ranges = config_dict.get("uwb_ranges", None)
            ground_truth = uwb_ranges.get("ground_truth", None)
            antennas_names = []
            for node, config in uwb_nodes.items():
                for antenna in config["names"]:
                    antennas_names.append(f"{node}_{antenna}")
            print(f"UWB nodes config: {uwb_nodes}")
            print(f"UWB ranges config: {uwb_ranges}")
            print(f"Robot antennas names: {antennas_names}")
            if uwb_ranges["type_measurement"] == "all":
                print(f"Measuring all to all")
                # Generate the combination of antennas
                measurements = list(permutations(antennas_names, 2))
                # Remove the permutations that are in the same robot (e.g. T01_A -> T01_B) since we do not want to calculate the distance between them
                measurements = [
                    elem
                    for elem in measurements
                    if elem[0].split("_")[0] != elem[1].split("_")[0]
                ]
                print(f"Measurements: {measurements}")
            else:
                print(f"Measuring {uwb_ranges['ground_truth']} to all")
                # Filter the antennas of the origin
                origin_antennas = [
                    antenna for antenna in antennas_names if ground_truth in antenna
                ]
                end_antennas = [
                    antenna
                    for antenna in antennas_names
                    if antenna not in origin_antennas
                ]
                # Generate the combination of antennas
                measurements = list(product(origin_antennas, end_antennas))
                # print(f"Origin antennas: {origin_antennas}")
                print(f"End antennas: {end_antennas}")
            # Generate names of the topics to publish or subscribe related to the distances
            topics_distances = []
            for origin, end in measurements:
                topics_distances.append(f"from_{origin}_to_{end}")
            print(f"Distance topics names: {topics_distances}")
            self.topics_distances = topics_distances
            # Generaet the names of the topics to publish or subscribe related to the global positions of the antennas
            topics_global_positions = []
            # Iterate over the robots in the system
            for node, config in uwb_nodes.items():
                # Append the subscriber to the list
                topics_global_positions.append(f"/{node}_antennas")
            print(f"Global position topic names: {topics_global_positions}")
            # Iterate to generate the odometry topics
            topics_odom = []
            for node, config in uwb_nodes.items():
                # Append the subscriber to the list
                topics_odom.append(f"/{node}/odom")
            self.topics_odom = topics_odom
            print(f"Odometry topics names: {topics_odom}")
            self.topics_opti = []

            # TODO: Ver que topics usar para las funciones de callback
            # TODO: Crear la lista de topics odom

        # Load the odometry topics from the bag
        self.subcribers_odom = []
        for idx, topic in enumerate(self.topics_odom):
            print(f"Topic: {topic}")
            # Create the subscriber
            odometry = self.create_subscription(
                Odometry, topic, self.read_odom(topic), qos_profile=self.qos_policy
            )
            # Append the subscriber to the list
            self.subcribers_odom.append(odometry)
        # Load the optitrack topics from the bag
        self.subcribers_opti = []
        for idx, topic in enumerate(self.topics_opti):
            print(f"Topic: {topic}")
            # Create the subscriber
            optitrack = self.create_subscription(
                PoseStamped,
                topic,
                self.read_opti(topic),
                qos_profile=self.qos_policy,
            )
            # Append the subscriber to the list
            self.subcribers_opti.append(optitrack)
        # Load the distances topics from the bag or the simulation
        self.subcribers_distances = []
        for idx, topic in enumerate(self.topics_distances):
            print(f"Topic: {topic}")
            # Create the subscriber
            distance = self.create_subscription(
                Float32, topic, self.read_distance(topic), qos_profile=self.qos_policy
            )
            # Append the subscriber to the list
            self.subcribers_distances.append(distance)
        # Declare the dictionary to save the data
        self.data = {key: None for key in self.topics_odom + self.topics_opti}
        # Create a pandas dataframe to save the data with the topics as columns
        # Generate the combination of topics and the coordinates (x, y, z)
        # self.cols = []
        # for topic in self.topics_odom + self.topics_opti:
        #     self.cols += [f"{topic}_x", f"{topic}_y", f"{topic}_z"]
        # print(f"Columns: {self.cols}")
        # Create the dataframe
        self.data_df = pd.DataFrame(columns=self.topics_odom + self.topics_opti + self.topics_distances)

        # Create the timer to process the data
        self.timer = self.create_timer(1.0, self.on_timer)

        # Create variables to save the csv file
        self.csv_file = (
            Path.cwd() / "src" / "uwb_real_trajectories" / f"positions{time()}.csv"
        )
        # Create the csv file
        with open(self.csv_file, "w") as file:
            first_line = ",".join(self.topics_odom + self.topics_opti + self.topics_distances)
            file.write(f"{first_line}\n")

    # Callback function for the odometry
    def read_odom(self, topic):
        # Create the callback function
        def callback(msg):
            # print(f"Topic: {topic}")
            # print(msg.pose.pose.position)
            self.data[topic] = msg.pose.pose.position
            # append(msg.pose.pose.position)
            # Save the data to the dataframe on the corresponding column
            self.data_df[topic] = [
                (
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                )
            ]

        return callback

    # Callback function for the optitrack
    def read_opti(self, topic):
        # Create the callback function
        def callback(msg):
            # print(f"Topic: {topic}")
            # print(msg)
            self.data[topic] = msg.pose
            # Save the data to the dataframe on the corresponding column
            self.data_df[topic] = [
                (
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                )
            ]

        return callback

    # Callback function for the distances
    def read_distance(self, topic):
        # Create the callback function
        def callback(msg):
            # print(f"Topic: {topic}")
            # print(msg)
            self.data[topic] = msg.data
            self.data_df[topic] = [msg.data]

        return callback

    def on_timer(self):
        # Add the data to the csv file
        with open(self.csv_file, "a") as f:
            writer = csv.writer(f)
            # Check if the dataframe is empty
            if not self.data_df.empty:
                # Get row from the dataframe
                row = self.data_df.iloc[0].values.tolist()
                print(f"Row: {row}")
                # Write the row in the csv file
                writer.writerow(row)


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    minimal_client = SaveData()

    try:
        minimal_client.get_logger().info("Starting node")

        rclpy.spin(minimal_client)

    except KeyboardInterrupt:
        pass
    finally:
        minimal_client.get_logger().info("Shutting down")
        minimal_client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
