import math
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import sys
from random import randint
import ast
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float32
from itertools import combinations, permutations, product

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


class FrameListener(Node):
    def __init__(self):
        super().__init__(f'tf2_listener_{randint(0, 1000)}')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

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
        self.declare_parameter('topics_to_publish', rclpy.Parameter.Type.STRING_ARRAY)
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
        self.topics_to_publish = self.get_parameter_or('topics_to_publish', None)
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
        self.get_logger().info(f"topics_to_publish: {self.topics_to_publish.value}")

        # Convert the nodes from string to dictionray
        self.nodes_config_dict = ast.literal_eval(self.nodes_config.value)
        self.get_logger().info(f"nodes_config converted: {self.nodes_config.value}")

        # Convert the measurements from string to list
        self.measurements_list = ast.literal_eval(self.measurements.value)
        self.get_logger().info(f"measurements converted: {self.measurements_list}")

        # Check if the pairs are all in the nodes config
        for pair in self.pairs.value:
            if pair not in self.nodes_config_dict:
                self.get_logger().error(f"Pair {pair} is not in the nodes_config")

        # Check if the measurements are between an origind and a destination or all to all
        # if self.ground_truth.value == 'all':
        #     self.get_logger().info(f"Measuring all to all")
        #     # Generate the combination of antennas
        #     self.measurements = list(permutations(self.antennas_names.value, 2))
        #     # Remove the permutations that are in the same robot (e.g. T01_A -> T01_B) since we do not want to calculate the distance between them
        #     self.measurements = [elem for elem in self.measurements if elem[0].split('_')[0] != elem[1].split('_')[0]]
        # else:
        #     self.get_logger().info(f"Measuring {self.ground_truth.value} to all")
        #     # Filter the antennas of the origin
        #     self.origin_antennas = [antenna for antenna in self.antennas_names.value if self.ground_truth.value in antenna]
        #     self.end_antennas = [antenna for antenna in self.antennas_names.value if antenna not in self.origin_antennas]
        #     self.get_logger().info(f"Origin antennas: {self.origin_antennas}")
        #     self.get_logger().info(f"End antennas: {self.end_antennas}")
        #     # Generate the combination of antennas
        #     self.measurements = list(product(self.origin_antennas, self.end_antennas))
        # self.get_logger().info(f"Measurements to calculate: {self.measurements}")

        # Create the buffer and the listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Generate dictionary of publishers
        # TODO verify if there is a way of doing this without the function generate_publishers_names, maybe using the parameter directly (measurements and topics_to_publish)
        self.publishers_ = self.generate_publishers_names()

        # Call on_timer function
        # TODO verify if we have to use the variable max_freq or duty_cycle
        self.timer = self.create_timer(1.0 / self.max_freq.value, self.on_timer)
        self.i = 0

        # TODO: Add the subscription to the ranges topic

    def generate_publishers_names(self):
        publishers = {}
        for topic in self.topics_to_publish.value:
            publishers[topic] = (self.create_publisher(Float32, topic, 10), Float32())
        return publishers

    def on_timer(self):
        # Iterate over the pairs of nodes to calculate the transforms
        self.i += 1
        if self.i == self.max_freq:
            self.i = 0
        # Get the index of the pair to calculate
        idx = self.i % len(self.measurements_list)
        # Obtain the origin and end from the variable self.measuments to generate the transforms (origin, end)
        to_frame_rel = self.measurements_list[idx][0]
        from_frame_rel = self.measurements_list[idx][1]
        # print(f'To: {to_frame_rel}, From: {from_frame_rel}')
        # Get the transform between the origin and the end
        try:
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
            # calculate the norm of the vector
            t_x = t.transform.translation.x
            t_y = t.transform.translation.y
            t_z = t.transform.translation.z
            # Publish the transform
            # self.tf_publisher_.publish(t)
            # Calculate the norm of the vector
            norm = np.linalg.norm([t_x, t_y, t_z])
            # Add noise to the norm
            norm += np.random.normal(self.mean_noise.value, self.std_dev_noise.value)
            # Publish the norm
            publisher_ = self.publishers_[f'from_{to_frame_rel}_to_{from_frame_rel}'][0]
            msg = self.publishers_[f'from_{to_frame_rel}_to_{from_frame_rel}'][1]
            msg.data = norm
            publisher_.publish(msg)
            # Print the information
            # print(
            #     f'to_frame_rel: {to_frame_rel} -> from_frame_rel: {from_frame_rel}')
            # print(f't: {t}')
            # print(f'transform: {t.transform}')
            # print(f'norm: {norm}')
            # print()
        except TransformException as ex:
            # self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            # exit(0)
            return
        return


def main(args=None):
    rclpy.init(args=args)
    print(args)

    node = FrameListener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
