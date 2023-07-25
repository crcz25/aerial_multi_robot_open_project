import math
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import nav_msgs.msg
import sys
import ast
from random import randint
from std_msgs.msg import Float32
from itertools import combinations, permutations, product
import numpy as np
import csv
import time
import pandas as pd
from ._localization import lse, mlt_tri_from_measurements_table
from pprint import pprint
from time import sleep

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from uwb_interfaces.msg import FloatDictionary, UwbRange

# This function is a stripped down version of the code in
# https://github.com/matthew-brett/transforms3d/blob/f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/euler.py
# Besides simplifying it, this version also inverts the order to return x,y,z,w, which is
# the way that ROS prefers it.
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class Plotter(Node):

    def __init__(self):
        super().__init__(f'plotter_{randint(0, 1000)}')
        self.qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Declare the parameters
        self.declare_parameter('nodes_config', rclpy.Parameter.Type.STRING)
        self.declare_parameter('ground_truth', rclpy.Parameter.Type.STRING)
        self.declare_parameter('max_freq', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('duty_cycle', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('mean', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('std_dev', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('pairs_to_measure', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('antennas', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('measurements', rclpy.Parameter.Type.STRING)
        self.declare_parameter('distances_to_subscribe', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('positions_to_subscribe', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('write_to_file', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('localization_method', rclpy.Parameter.Type.STRING)
        # Get the parameters
        self.nodes_config = self.get_parameter_or('nodes_config', None)
        self.ground_truth = self.get_parameter_or('ground_truth', None)
        self.max_freq = self.get_parameter_or('max_freq', None)
        self.duty_cycle = self.get_parameter_or('duty_cycle', None)
        self.mean_noise = self.get_parameter_or('mean', None)
        self.std_dev_noise = self.get_parameter_or('std_dev', None)
        self.pairs = self.get_parameter_or('pairs_to_measure', None)
        self.antennas_names = self.get_parameter_or('antennas', None)
        self.measurements = self.get_parameter_or('measurements', None)
        self.distances_to_subscribe = self.get_parameter_or('distances_to_subscribe', None)
        self.positions_to_subscribe = self.get_parameter_or('positions_to_subscribe', None)
        self.write_to_file = self.get_parameter_or('write_to_file', False)
        self.localization_method = self.get_parameter_or('localization_method', None)
        # Print the parameters
        # self.get_logger().info(f"nodes_config: {self.nodes_config.value}")
        # self.get_logger().info(f"ground_truth: {self.ground_truth.value}")
        # self.get_logger().info(f"max_freq: {self.max_freq.value}")
        # self.get_logger().info(f"duty_cycle: {self.duty_cycle.value}")
        # self.get_logger().info(f"mean: {self.mean_noise.value}")
        # self.get_logger().info(f"std_dev: {self.std_dev_noise.value}")
        # self.get_logger().info(f"pairs: {self.pairs.value}")
        # self.get_logger().info(f"antennas: {self.antennas_names.value}")
        # self.get_logger().info(f"measurements: {self.measurements.value}")
        # self.get_logger().info(f"distances_to_subscribe: {self.distances_to_subscribe.value}")
        # self.get_logger().info(f"positions_to_subscribe: {self.positions_to_subscribe.value}")
        # self.get_logger().info(f"write_to_file: {self.write_to_file.value}")
        # self.get_logger().info(f"type(write_to_file): {type(self.write_to_file.value)}")
        # self.get_logger().info(f"localization_method: {self.localization_method.value}")

        # Convert the nodes from string to dictionray
        self.nodes_config_dict = ast.literal_eval(self.nodes_config.value)
        # self.get_logger().info(f"nodes_config converted: {self.nodes_config.value}")

        # Convert the measurements from string to list
        self.measurements_list = ast.literal_eval(self.measurements.value)
        # self.get_logger().info(f"measurements converted: {self.measurements_list}")


        # Generate the list of subscribers for the UWB distances
        self.subscribers_distances = []
        self.ranges_ = {}
        # Iterate over the topics to subscribe
        for idx, topic in enumerate(self.distances_to_subscribe.value):
        # for idx, (origin, end) in enumerate(self.measurements_list):
            # Create the subscriber
            uwb_distance_subs = self.create_subscription(Float32, topic, self.read_distane(topic), qos_profile=self.qos_policy)
            # Append the subscriber to the list
            self.subscribers_distances.append(uwb_distance_subs)
        # Initialize the corresponding ranges
        # self.ranges_ = np.zeros((len(self.distances_to_subscribe.value), 1))

        # Generate the list of subscribers for the global positions
        self.subscribers_positions = []
        self.global_positions_ = {}
        # Iterate over the robots in the system
        for topic in self.positions_to_subscribe.value:
            # Create the subscriber
            position_subs = self.create_subscription(TransformStamped, topic, self.read_positions(), qos_profile=self.qos_policy)
            # Append the subscriber to the list
            self.subscribers_positions.append(position_subs)

        # Initialize the corresponding global positions
        # self.global_positions_ = np.zeros((len(self.antennas_names.value), 3))

        # Create the buffer and the listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Create the timer to read the positions
        # self.timer_positions = self.create_timer(1.0 / (self.max_freq.value * 1.5), self.read_positions)

        # Create the timer to save the data
        self.timer = self.create_timer(1.0, self.on_timer)

        # Get the global positions from the ground truth
        self.gt_idx = [i for i, antenna in enumerate(self.antennas_names.value) if self.ground_truth.value in antenna]
        # Get the antennas from the ground truth
        self.gt_antennas = [antenna for antenna in self.antennas_names.value if self.ground_truth.value in antenna]
        # Get the rest of antennas
        self.rest_antennas = [antenna for antenna in self.antennas_names.value if antenna not in self.gt_antennas]
        # Get the corresponding index of each topic for each antenna
        self.topcis_idx = {}
        for antenna in self.rest_antennas:
            self.topcis_idx[antenna] = [i for i, topic in enumerate(self.distances_to_subscribe.value) if antenna in topic]

        # Create empy dataframe to store all the data recieved from the suscriptions
        # from_T01_A_to_T02_A_range,from_T01_A_to_T02_B_range,from_T01_A_to_tello1_A_range,from_T01_B_to_T02_A_range,from_T01_B_to_T02_B_range,from_T01_B_to_tello1_A_range,from_T01_C_to_T02_A_range,from_T01_C_to_T02_B_range,from_T01_C_to_tello1_A_range,GT_T01_A_x,GT_T01_A_y,GT_T01_A_z,GT_T01_B_x,GT_T01_B_y,GT_T01_B_z,GT_T01_C_x,GT_T01_C_y,GT_T01_C_z,T02_A_x,T02_A_y,T02_A_z,T02_B_x,T02_B_y,T02_B_z,tello1_A_x,tello1_A_y,tello1_A_z
        self.cols = []
        # Add the topics from the distances of the ground truth and the nodes
        for topic in self.distances_to_subscribe.value:
            self.cols.append(topic)
        # Add the antennas names
        for antenna in self.antennas_names.value:
                self.cols.append(antenna)
        # self.get_logger().info(f"cols: {self.cols}")
        # Create the dataframe with default values as 0.0
        self.data_df = pd.DataFrame(columns=self.cols)
        # self.get_logger().info(f"data_df:")
        # print(self.data_df)

        # Create the variables to save the data in a csv file
        self.now = time.time()
        if self.write_to_file.value:
            # self.get_logger().info(f"Creating csv file")
            with open(f'measurements_{self.now}.csv', 'w') as f:
                writer = csv.writer(f)
                cols = []
                for topic in self.distances_to_subscribe.value:
                    cols.append(f'{topic}_range')
                for antenna in self.antennas_names.value:
                    # Check if the antenna has the name of the ground truth robot:
                    if antenna.startswith(self.ground_truth.value):
                        # Append GT to the beginning of the name
                        cols.append(f'GT_{antenna}')
                    else:
                        cols.append(f'{antenna}')
                writer.writerow(cols)
        
        # Create publisher for the range measurements
        self.publisher_range = self.create_publisher(FloatDictionary, 'range_measurements', 10)

    # Callback function for the subscriber of the topic
    def read_distane(self, topic):
        # Create the callback function for the subscriber of the topic
        def callback(msg):
            # Update the corresponding range in the list
            self.ranges_[topic] = msg.data
            # Save the range in the dataframe
            self.data_df.loc[0, topic] = msg.data
            # Print the range in the terminal
            # self.get_logger().info(f"Range of {topic}: {self.ranges_[topic]}")
        # Return the callback function
        return callback
    
    def read_positions(self):
        # Create the callback function for the subscriber of the topic
        def callback(msg):
            # Save the global position of each antenna
            self.global_positions_[msg.child_frame_id] = [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z]
            # Save the global position of the antenna in the dataframe
            self.data_df.loc[0, f'{msg.child_frame_id}'] = [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z]
            # Print the global position of the antenna
            # self.get_logger().info(f"Global position of {msg.child_frame_id}: {self.global_positions_[msg.child_frame_id]}")
        # Return the callback function
        return callback

    def on_timer(self):
        # Process distance measurements
        # self.get_logger().info(f"\n\nProcessing measurements")
        # self.get_logger().info(f"Ranges:")
        # pprint(self.ranges_)
        # self.get_logger().info(f"Positions:")
        # pprint(self.global_positions_)
        # pprint(self.data_df)

        # Process the data only if there is no na value in the dataframe or if the dataframe is not empty
        if not self.data_df.isnull().values.any() and not self.data_df.empty:
            # Construct the list of positions from the ground truth antennas using the dataframe
            for antenna in self.rest_antennas:
                # Get the ranges that contain the antenna in the name of the column _to_antenna
                cols = self.data_df.columns.map(lambda x: f'to_{antenna}' in x)
                # Get the ranges of the antennas
                ranges_antenna = self.data_df.loc[:, cols].values.flatten()
                # Convert the ranges to a numpy array
                # ranges_antenna = np.array(ranges_antenna)
                # Get the positions of the ground truth antennas
                gt_pos = self.data_df.loc[:, self.gt_antennas].values.flatten()
                # Convert the positions to a numpy array
                # gt_pos = np.array(gt_pos)

                # self.get_logger().info(f"Ranges:")
                # print(ranges_antenna)
                # self.get_logger().info(f"Positions:")
                # print(gt_pos)

                # Check the localization method
                if self.localization_method.value == "lse":
                    # Calculate the estimated position
                    estimated_position, err = lse(gt_pos, ranges_antenna)
                    # Print the estimated position
                    # self.get_logger().info(f"Estimated position of {antenna}: {estimated_position}, error: {err}")
            if self.localization_method.value == "trilat":
                n_ranges = len(self.distances_to_subscribe.value)

                measurements_table = self.data_df.to_numpy()[:, :n_ranges]
                measurements_cols = self.data_df.columns.to_list()[:n_ranges]

                estimated_positions = mlt_tri_from_measurements_table(
                    measurements_table=measurements_table,
                    measurements_cols_names=measurements_cols,
                    origin_antenna_1=self.antennas_names.value[0],
                    origin_antenna_2=self.antennas_names.value[1],
                    all_antennas=self.antennas_names.value,
                    range_suffix=''
                )

        # Save the data in a csv file
        if self.write_to_file.value:
            with open(f'measurements_{self.now}.csv', 'a') as f:
                writer = csv.writer(f)
                # Check if the dataframe is empty
                if not self.data_df.empty:
                    # Get row from the dataframe
                    row = self.data_df.iloc[0].values.tolist()
                    # Write the row in the csv file
                    writer.writerow(row)
        pass

def main(args=None):
    rclpy.init(args=args)

    node = Plotter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()