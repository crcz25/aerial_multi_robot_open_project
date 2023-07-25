import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import Point, Quaternion, Twist
import nav_msgs.msg
import sys
import ast
import pandas as pd
from random import randint
import re
from tello_msgs.srv import TelloAction

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

class RobotTrajectory(Node):

    def __init__(self):
        super().__init__(f'trajectories_{randint(0, 1000)}')
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Declare the parameters
        self.declare_parameter('nodes_config', rclpy.Parameter.Type.STRING)
        # Get the parameters
        self.nodes_config = self.get_parameter_or('nodes_config', None)
        # Print the parameters
        self.get_logger().info(f"nodes_config: {self.nodes_config.value}")
        self.nodes_config_dict = ast.literal_eval(self.nodes_config.value)
        self.get_logger().info(f"nodes_config converted: {self.nodes_config.value}")

        # Takeoff Control Simulator
        self.cli = self.create_client(TelloAction, '/tello1/tello_action')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()


        # Generate the topics for the odometry
        self.odom_topics = []
        self.odom_data = {}
        self.publisher_twists = {}
        # Iterate over the robots in the system
        for idx, robot in enumerate(self.nodes_config_dict.keys()):
            self.get_logger().info(f"Creating topic: {robot}")
            # Create the topics name
            odom_name = f'{robot}/odom'
            twist_name = f'{robot}/cmd_vel'
            # Create the subscriber for the odometry
            odom = self.create_subscription(Odometry, odom_name, self.read_odom(odom_name, idx), qos_profile=qos_policy)
            # Create the publisher for the twist
            publisher_twist = self.create_publisher(Twist, twist_name, 1)
            # Append the subscriber to the list
            self.odom_topics.append(odom)
            self.publisher_twists[robot] = publisher_twist

        # List of drones
        reg = re.compile(r'^tello.*')
        self.drones = list(filter(reg.match, self.nodes_config_dict.keys()))
        # List of Turtlebots
        reg = re.compile(r'^T\d*')
        self.turtlebots = list(filter(reg.match, self.nodes_config_dict.keys()))

        # Create the timer
        self.create_timer(0.1, self.main_node)
        # Trajectories
        self.start = [True] * len(self.nodes_config_dict.keys())
        self.trajectory_odom = []
        self.error = []
        self.center = [Point(x=0.0 , y=0.0, z=0.0)] * len(self.nodes_config_dict.keys())
        self.delta_theta = 0.1
        self.radius = 1.0
        self.in_center = False
        self.z_steps = 1.0
        # Create the twist message
        self.destination_twist = Twist()

    # Callback function for the subscriber of the topic
    def read_odom(self, topic, idx):
        # Create the callback function for the subscriber of the topic
        def callback(msg):
            # Get the position
            position = msg.pose.pose.position
            # Get the orientation
            orientation = msg.pose.pose.orientation
            # Calculate theta
            theta = np.arctan2(position.y - self.center[idx].y, position.x - self.center[idx].x)
            # Save the odometry data
            self.odom_data[topic] = {'pos': position, 'ori': orientation, 'theta': theta}
            # Print the odometry
            # self.get_logger().info(f"Odometry from {topic}: {self.odom_data[topic]}")
            return msg
        # Return the callback function
        return callback

    def polar_to_cartesian(self, r, theta, center):
        x = r * np.cos(theta) + center[0]
        y = r * np.sin(theta) + center[1]
        return x, y

    def send_request(self, cmd):
        self.get_logger().info('Sending request {}'.format(cmd))
        self.req.cmd = cmd
        self.future = self.cli.call_async(self.req)

    def move_drone(self, drone, topic, idx):
        # self.get_logger().info(f"Moving drone")

        # Save the center on the first iteration
        if self.start[idx]:
            center = self.odom_data[topic]['pos']
            self.center[idx] = center
            self.start[idx] = False

        # Get current position
        drone_pos = np.array([self.odom_data[topic]['pos'].x, self.odom_data[topic]['pos'].y])
        # Get the center
        center_pos = np.array([self.center[idx].x, self.center[idx].y])
        # Calculate the distance to the center
        distance = np.linalg.norm(drone_pos - center_pos) - self.radius

        # Calculate the new theta
        new_theta = 0
        if abs(distance) >  0.2 :
            # self.get_logger().info('Out of range, moving to the new center (radius)')
            new_theta = self.odom_data[topic]['theta']
        else:
            # self.get_logger().info('In range, moving in a circle')
            new_theta = self.odom_data[topic]['theta'] + self.delta_theta

        # Calculate X, Y of the new position
        x_obj, y_obj = self.polar_to_cartesian(self.radius, new_theta, center_pos)
        obj = np.array([x_obj, y_obj])
        dir_x, dir_y = (obj - drone_pos) / np.linalg.norm(obj - drone_pos)

        self.destination_twist.linear.x = dir_x * 0.1
        self.destination_twist.linear.y = dir_y * 0.1

        # Publish the twist
        self.publisher_twists[drone].publish(self.destination_twist)

        # print(f"Drone: {drone}")
        # print(f"Current position: {drone_pos}")
        # print(f"Center: {center_pos}")
        # print(f"Current distance: {distance}")
        # print(f"old theta: {self.odom_data[topic]['theta']}, new theta: {new_theta}")
        return

    def move_turtlebot(self, turtle, topic, idx):
        # self.get_logger().info(f"Moving turtlebot")
        # Save the center on the first iteration
        if self.start[idx]:
            center = self.odom_data[topic]['pos']
            self.center[idx] = center
            self.start[idx] = False

        # Get current position
        turtle_pos = np.array([self.odom_data[topic]['pos'].x, self.odom_data[topic]['pos'].y])
        # Get the center
        center_pos = np.array([self.center[idx].x, self.center[idx].y])

        # Publish the twist
        self.publisher_twists[turtle].publish(self.destination_twist)

        # self.get_logger().info(f"Center: {center_pos}")
        # self.get_logger().info(f"Position: {turtle_pos}")
        return

    def move_robots(self):
        # self.get_logger().info(f"Moving robots")
        # Iterate over the list of robots
        for idx, (robot, center) in enumerate(zip(self.nodes_config_dict.keys(), self.center)):
            topic = f'{robot}/odom'
            # Check if the drone is in the odometry data
            if topic in self.odom_data.keys():
                # Check if the robot is a drone or a turtlebot
                if robot in self.drones:
                    self.move_drone(robot, topic, idx)
                elif robot in self.turtlebots:
                    self.move_turtlebot(robot, topic, idx)

        return

    def main_node(self):
        self.move_robots()

def main(args=None):
    rclpy.init(args=args)

    node = RobotTrajectory()
    
    try:
        node.send_request('takeoff')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_request('land')
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()