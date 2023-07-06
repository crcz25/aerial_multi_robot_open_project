import math
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import sys
import ast
from random import randint
from std_msgs.msg import Float32
from itertools import combinations, permutations, product
import numpy as np
import csv

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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
        self.declare_parameter('topics_to_subscribe', rclpy.Parameter.Type.STRING_ARRAY)
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
        self.topics_to_subscribe = self.get_parameter_or('topics_to_subscribe', None)
        # Print the parameters
        self.get_logger().info(f"nodes_config: {self.nodes_config.value}")
        self.get_logger().info(f"robot_name: {self.ground_truth.value}")
        self.get_logger().info(f"max_freq: {self.max_freq.value}")
        self.get_logger().info(f"duty_cycle: {self.duty_cycle.value}")
        self.get_logger().info(f"mean: {self.mean_noise.value}")
        self.get_logger().info(f"std_dev: {self.std_dev_noise.value}")
        self.get_logger().info(f"pairs: {self.pairs.value}")
        self.get_logger().info(f"antennas: {self.antennas_names.value}")
        self.get_logger().info(f"measurements: {self.measurements.value}")
        self.get_logger().info(f"topics_to_subscribe: {self.topics_to_subscribe.value}")

        # Convert the measurements from string to list
        self.measurements_list = ast.literal_eval(self.measurements.value)
        self.get_logger().info(f"measurements converted: {self.measurements_list}")

        self.measurements = []

        # Generate the list of subscribers
        self.subscribers_ = []
        # Iterate over the topics to subscribe
        for idx, topic in enumerate(self.topics_to_subscribe.value):
            # Create the subscriber
            uwb_distance_subs = self.create_subscription(Float32, topic, self.read_distane(idx), qos_profile=self.qos_policy)
            # Append the subscriber to the list
            self.subscribers_.append(uwb_distance_subs)
        # Initialize the corresponding ranges
        self.ranges_ = np.zeros((len(self.topics_to_subscribe.value), 1))
        # Initialize the corresponding global positions
        self.global_positions_ = np.zeros((len(self.antennas_names.value), 3))

        # Create the buffer and the listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_positions = self.create_timer(1.0 / (self.max_freq.value * 1.5), self.read_positions)
        # Create the buffer and the listener
        self.timer = self.create_timer(1.0 / self.max_freq.value, self.on_timer)

        # Create the variables to save the data in a csv file
        with open(f'measurements.csv', 'w') as f:
            writer = csv.writer(f)
            cols = []
            for topic in self.topics_to_subscribe.value:
                cols.append(f'{topic}_range')
            for antenna in self.antennas_names.value:
                cols.append(f'{antenna}_x')
                cols.append(f'{antenna}_y')
                cols.append(f'{antenna}_z')
            writer.writerow(cols)


    # Callback function for the subscriber of the topic
    def read_distane(self, idx):
        # Create the callback function for the subscriber of the topic
        def callback(msg):
            # Update the corresponding range in the list
            self.ranges_[idx] = msg.data
            # Print the range in the terminal
            # self.get_logger().info(f"Range {idx} topic {self.topics_to_subscribe.value[idx]}: {msg.data}")
        # Return the callback function
        return callback
    
    def read_positions(self):
        try:
            # Iterate over the antennas to find each transform (global position)
            for idx, antenna in enumerate(self.antennas_names.value):
                # Get the transform from the antenna to the world
                t = self.tf_buffer.lookup_transform('world', f'{antenna}', rclpy.time.Time())
                # Save the global position of the antenna
                self.global_positions_[idx] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                # Print the global position of the antenna
                # self.get_logger().info(f"Antenna {antenna}: {t.transform.translation.x}, {t.transform.translation.y}, {t.transform.translation.z}")
                # self.get_logger().info(f"Antenna {antenna}: {self.global_positions_[idx]}")
        except Exception as e:
            self.get_logger().info(f"Exception: {e}")
            return
        return

    def on_timer(self):
        # Save to csv
        self.get_logger().info(f"Saving to csv")
        self.get_logger().info(f"Ranges: \n{self.ranges_}")
        self.get_logger().info(f"Positions: \n{self.global_positions_}")
        
        # Save the data in a csv file
        with open(f'measurements.csv', 'a') as f:
            writer = csv.writer(f)
            cols = []
            for range in self.ranges_:
                cols.append(range.item())
            for position in self.global_positions_:
                for coord in position:
                    cols.append(coord)
            writer.writerow(cols)

        pass

def main(args=None):
    rclpy.init(args=args)
    print(args)

    node = Plotter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()