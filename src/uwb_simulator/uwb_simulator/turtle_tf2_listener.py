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
        super().__init__(f'turtle_tf2_listener_{randint(0, 1000)}')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # Get parameters from the parameter server
        if len(sys.argv) > 1:
            self.ground_truth = sys.argv[1]
            self.nodes_config = ast.literal_eval(sys.argv[2])
            self.max_freq = float(sys.argv[3])
            self.duty_cycle = float(sys.argv[4])
            self.pairs = sys.argv[5].split('_')
            self.get_logger().info(f"robot_name: {self.ground_truth}")
            self.get_logger().info(f"nodes_config: {self.nodes_config}")
            self.get_logger().info(f"max_freq: {self.max_freq}")
            self.get_logger().info(f"duty_cycle: {self.duty_cycle}")
            self.get_logger().info(f"pairs: {self.pairs}")
        else:
            self.ground_truth = 'T01'
            self.nodes_config = {'T01': {'num_antennas': 4, 'names': ['A', 'B', 'C', 'D']}, 'T02': {'num_antennas': 2, 'names': ['A', 'B']}}
            self.max_freq = 400.0
            self.duty_cycle = 1.0
            self.pairs =  "T02".split('_')
            self.get_logger().info("No arguments received")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Generate the names of the antennas
        self.origin_antennas = []
        self.end_antennas = []
        self.measurements = []
        self.generate_antennas_names()

        # Generate dictionary of publishers
        self.publishers_ = self.generate_publishers_names()

        # Declare and acquire target frame
        self.target_frame = f'/{self.ground_truth}/tf/base_footprint'
        # Call on_timer function
        # TODO verify if we have to use the variable max_freq or duty_cycle
        self.timer = self.create_timer(1.0/self.max_freq, self.on_timer)
        self.i = 0
        # TODO: Add the subscription to the ranges topic

    def generate_publishers_names(self):
        publishers = {}
        for origin, end in self.measurements:
            publishers[f'from_{origin}_to_{end}'] = (self.create_publisher(Float32, f'from_{origin}_to_{end}', 10), Float32())
        return publishers

    def generate_antennas_names(self):
        origin_config = self.nodes_config[self.ground_truth]
        self.origin_antennas = [f'{self.ground_truth}/tf/{antenna}' for antenna in origin_config['names']]
        for elem in self.pairs:
            end_config = self.nodes_config[elem]
            self.end_antennas.append([f'{elem}/tf/{antenna}' for antenna in end_config['names']])
        self.end_antennas = [item for sublist in self.end_antennas for item in sublist]
        # Generate the combination of antennas
        self.measurements = [(origin, end) for origin in self.origin_antennas for end in self.end_antennas]

        # self.get_logger().info('------------------')
        # self.get_logger().info(f'origin: {self.ground_truth}')
        # self.get_logger().info(f'origin_config: {origin_config}')
        # self.get_logger().info(f'origin_antennas: {self.origin_antennas}')
        # self.get_logger().info(f'end_antennas: {self.end_antennas}')
        # self.get_logger().info(f'measurements: {self.measurements}')
        # self.get_logger().info('------------------')

    def on_timer(self):
        # Iterate over the pairs of nodes to calculate the transforms
        self.i += 1
        if self.i == self.max_freq:
            self.i = 0
        # Get the index of the pair to calculate
        idx= self.i % len(self.measurements)
        # Obtain the origin and end from the variable self.measuments to generate the transforms (origin, end)
        to_frame_rel = self.measurements[idx][0]
        from_frame_rel = self.measurements[idx][1]
        # Get the transform between the origin and the end
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            # calculate the norm of the vector
            t_x = t.transform.translation.x
            t_y = t.transform.translation.y
            t_z = t.transform.translation.z
            # Calculate the norm of the vector
            norm = np.linalg.norm([t_x, t_y, t_z])
            # Add noise to the norm
            norm += np.random.normal(0, 10)
            # Publish the norm
            publisher_ = self.publishers_[f'from_{to_frame_rel}_to_{from_frame_rel}'][0]
            msg = self.publishers_[f'from_{to_frame_rel}_to_{from_frame_rel}'][1]
            msg.data = norm
            publisher_.publish(msg)
            # Print the information
            print(f'to_frame_rel: {to_frame_rel} -> from_frame_rel: {from_frame_rel}')
            print(f't: {t}')
            print(f'transform: {t.transform}')
            print(f'norm: {norm}')
            print()
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

                # print(f'origin: {origin}, end: {end}')
        pass

def main(args=None):
    rclpy.init(args=args)
    print(args)

    node = FrameListener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()