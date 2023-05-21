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
            self.robot_name = sys.argv[1]
            self.nodes_config = ast.literal_eval(sys.argv[2])
            self.max_freq = float(sys.argv[3])
            self.duty_cycle = float(sys.argv[4])
            self.ranges = sys.argv[5].split('+')
            self.get_logger().info(f"robot_name: {self.robot_name}")
            self.get_logger().info(f"nodes_config: {self.nodes_config}")
            self.get_logger().info(f"max_freq: {self.max_freq}")
            self.get_logger().info(f"duty_cycle: {self.duty_cycle}")
            self.get_logger().info(f"ranges: {self.ranges}")
        else:
            self.robot_name = 'T01'
            self.nodes_config = {'T01': {'num_antennas': 4, 'names': ['A', 'B', 'C', 'D']}, 'T02': {'num_antennas': 2, 'names': ['A', 'B']}}
            self.max_freq = 400.0
            self.duty_cycle = 1.0
            self.ranges = ['T01_T02']
            self.get_logger().info("No arguments received")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Declare and acquire target frame
        self.target_frame = f'/{self.robot_name}/tf/base_footprint'
        # Call on_timer function
        # TODO verify if we have to use the variable max_freq or duty_cycle
        self.timer = self.create_timer(1.0/self.max_freq, self.on_timer)

    def on_timer(self):
        # Iterate over the ranges to calculate the transforms
        for i in range(len(self.ranges)):
            if self.ranges[i] != '':
                # Get the origin and end of the range
                origin = self.ranges[i].split('_')[0]
                end = self.ranges[i].split('_')[1]
                # Get the configuration of the origin and end
                origin_config = self.nodes_config[origin]
                end_config = self.nodes_config[end]
                # Generate antennas names
                origin_antennas = [f'{origin}/tf/{antenna}' for antenna in origin_config['names']]
                end_antennas = [f'{end}/tf/{antenna}' for antenna in end_config['names']]
                print(f'origin: {origin}')
                print(f'origin_config: {origin_config}')
                print(f'end: {end}')
                print(f'end_config: {end_config}')
                print(f'origin_antennas: {origin_antennas}')
                print(f'end_antennas: {end_antennas}')
                print('------------------')
                # Iterate over the antenas of the origin to calculate the transforms between the origin and the end
                for to_frame_rel in origin_antennas:
                    for from_frame_rel in end_antennas:
                        # Get the transform between the origin and the end
                        try:
                            t = self.tf_buffer.lookup_transform(
                                to_frame_rel,
                                from_frame_rel,
                                rclpy.time.Time())
                            print(f'to_frame_rel: {to_frame_rel} -> from_frame_rel: {from_frame_rel}')
                            print(f't: {t}')
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