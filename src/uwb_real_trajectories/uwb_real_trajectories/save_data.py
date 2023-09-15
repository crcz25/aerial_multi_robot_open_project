#! /usr/bin/python
import argparse
import sys
from pathlib import Path
from random import randint

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
from std_msgs.msg import Empty
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
            pass
        else:
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
            # Declare the dictionary to save the data
            self.data = {key: [] for key in self.topics_odom + self.topics_opti}
            # Create a pandas dataframe to save the data with the topics as columns
            self.data_df = pd.DataFrame(columns=self.topics_odom + self.topics_opti)

        # Create the timer to process the data
        self.timer = self.create_timer(1.0, self.on_timer)

    # Callback function for the odometry
    def read_odom(self, topic):
        # Create the callback function
        def callback(msg):
            # print(f"Topic: {topic}")
            # print(msg)
            self.data[topic].append(msg.pose.pose.position)
            # Save the data to the dataframe on the corresponding column
            self.data_df[topic] = [(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            )]

        return callback

    # Callback function for the optitrack
    def read_opti(self, topic):
        # Create the callback function
        def callback(msg):
            # print(f"Topic: {topic}")
            # print(msg)
            self.data[topic] = msg.pose
            # Save the data to the dataframe on the corresponding column
            self.data_df[topic] = [(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            )]

        return callback


    def on_timer(self):
        from pprint import pprint
        pprint(self.data)
        print(self.data_df)
        # plot the data from the dictionary
        plt.figure(1)
        plt.clf()
        for topic in self.topics_odom:
            plt.plot(
                [data.x for data in self.data[topic]],
                [data.y for data in self.data[topic]],
                label=topic,
            )
        plt.legend()
        plt.show(block=False)
        plt.pause(0.02)
        plt.clf()


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
