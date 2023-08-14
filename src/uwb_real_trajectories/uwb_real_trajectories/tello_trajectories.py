#! /usr/bin/python
import argparse
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from random import randint

import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tello_msgs.srv import TelloAction

from tf_transformations import euler_from_quaternion


class Tello(Node):
    def __init__(self):
        # Generate a node with a random name number
        super().__init__(f"Tello_trajectory_{randint(0, 1000)}")
        # Declare the parameters
        self.declare_parameter("n_loop", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("trajectory", rclpy.Parameter.Type.STRING)
        self.declare_parameter("sim", rclpy.Parameter.Type.BOOL)
        # Get the parameters
        self.n_loop = self.get_parameter_or("n_loop", None)
        self.trajectory = self.get_parameter_or("trajectory", None)
        self.sim = self.get_parameter_or("sim", None)
        # Set the loop number
        if self.n_loop:
            self.n_loop = self.n_loop.value

        # Print the parameters
        self.get_logger().info(f"n_loop: {self.n_loop}")
        self.get_logger().info(f"trajectory: {self.trajectory.value}")
        self.get_logger().info(f"sim: {self.sim.value}")

        # Create topics
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Configure the parameters and topics according to the sim
        self.odom_sub = self.create_subscription(
            Odometry, "/tello/odom", self.odom_callback, qos_profile=qos_policy
        )
        if self.sim.value:
            self.publisher_twist = self.create_publisher(Twist, "/tello/cmd_vel", 1)
            # Takeoff Control Simulator
            self.cli = self.create_client(TelloAction, "/tello/tello_action")
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("service not available, waiting again...")
            self.req = TelloAction.Request()
        else:
            # self.mocap_sub = self.create_subscription(PoseStamped, '/vrpn_client_node/newtello/pose', self.mocap_callback, qos_profile=qos_policy)
            self.publisher_twist = self.create_publisher(Twist, "/control", 1)
            self.publisher_takeoff = self.create_publisher(Empty, "/takeoff", 1)
            self.publisher_land = self.create_publisher(Empty, "/land", 1)

        # Create Twist publisher
        self.twist = Twist()

        self.create_timer(0.1, self.main_node)
        self.idx = 0

        # Control variables
        self.start = True
        self.reached_goal = False
        self.centered = False
        self.theta = 0.0

        if self.trajectory.value == "square":
            self.waypoints = [
                # [0.0, 0.0],
                [2.0, 0.0],
                [2.0, 2.0],
                [0.0, 2.0],
                [0.0, 0.0],
            ]
        elif self.trajectory.value == "circle":
            self.waypoints = [
                [1.5, 0.0],
                [1.4815325108927064, 0.23465169756034632],
                [1.4265847744427305, 0.4635254915624212],
                [1.3365097862825515, 0.6809857496093201],
                [1.2135254915624214, 0.8816778784387096],
                [1.0606601717798214, 1.0606601717798214],
                [0.8816778784387097, 1.2135254915624212],
                [0.6809857496093202, 1.3365097862825515],
                [0.4635254915624213, 1.4265847744427305],
                [0.23465169756034637, 1.4815325108927064],
                [9.184850993605148e-17, 1.5],
                [-0.23465169756034587, 1.4815325108927067],
                [-0.46352549156242107, 1.4265847744427307],
                [-0.68098574960932, 1.3365097862825517],
                [-0.8816778784387094, 1.2135254915624214],
                [-1.0606601717798212, 1.0606601717798214],
                [-1.2135254915624212, 0.8816778784387097],
                [-1.3365097862825515, 0.6809857496093203],
                [-1.4265847744427305, 0.46352549156242134],
                [-1.4815325108927064, 0.23465169756034648],
                [-1.5, 1.8369701987210297e-16],
                [-1.4815325108927064, -0.23465169756034615],
                [-1.426584774442731, -0.4635254915624204],
                [-1.3365097862825517, -0.6809857496093199],
                [-1.2135254915624216, -0.8816778784387094],
                [-1.0606601717798214, -1.0606601717798212],
                [-0.8816778784387098, -1.2135254915624212],
                [-0.6809857496093203, -1.3365097862825515],
                [-0.4635254915624214, -1.4265847744427302],
                [-0.2346516975603466, -1.4815325108927064],
                [-2.755455298081545e-16, -1.5],
                [0.234651697560346, -1.4815325108927064],
                [0.46352549156242095, -1.4265847744427307],
                [0.6809857496093199, -1.3365097862825517],
                [0.8816778784387092, -1.2135254915624216],
                [1.060660171779821, -1.0606601717798214],
                [1.213525491562421, -0.8816778784387098],
                [1.3365097862825515, -0.6809857496093205],
                [1.4265847744427302, -0.4635254915624215],
                [1.4815325108927064, -0.23465169756034668],
                [1.5, 0.0],
            ]

    def send_request(self, cmd):
        self.get_logger().info("Sending request {}".format(cmd))
        self.req.cmd = cmd
        self.future = self.cli.call_async(self.req)

    def take_off(self):
        self.get_logger().info("Taking off")
        self.publisher_takeoff.publish(Empty())

    def land(self):
        self.get_logger().info("Landing")
        self.publisher_land.publish(Empty())

    def odom_callback(self, msg):
        # self.get_logger().info("Saving odometry")
        # Save the current odometry
        self.odometry = msg
        orientation = msg.pose.pose.orientation
        if self.sim.value:
            # self.trajectory_odom.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
            _, _, self.theta = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
        else:
            # self.trajectory_odom.append([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
            _, _, self.theta = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
        # self.get_logger().info(f"Odometry: {msg}")

    def rotate(self, step):
        # self.get_logger().info(f"Rotating at {step} steps")
        self.twist.angular.z = step
        self.twist.linear.x = 0.0
        self.publisher_twist.publish(self.twist)

    def move_forward(self, step):
        # self.get_logger().info(f"Moving forward at {step} steps")
        self.twist.linear.x = step
        self.twist.angular.z = 0.0
        self.publisher_twist.publish(self.twist)

    def stop(self):
        # self.get_logger().info(f"Stopping")
        self.twist.angular.z = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.publisher_twist.publish(self.twist)

    def rotate_goal(self, waypoint):
        # Get the current position
        curr_pos = np.array(
            [self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y]
        )
        # Compute the vector between the current position and the waypoint
        # Calculate the vector
        pos_vector = waypoint - curr_pos
        # Calculate the distance to the waypoint
        dist_goal = np.linalg.norm(pos_vector)
        # Calculate the unit vector
        u_vector = pos_vector / dist_goal
        # Calculate the angle between the current position and the waypoint
        angle = np.arctan2(u_vector[1], u_vector[0])
        # Calculate the angle error
        angle_error = angle - self.theta
        angle_steps = 1.0

        self.get_logger().info(f"Rotate goal")
        self.get_logger().info(f"Current position: {curr_pos}")
        self.get_logger().info(f"Goal position: {waypoint}")
        self.get_logger().info(f"Distance to goal: {dist_goal}")
        self.get_logger().info(f"Unit vector: {u_vector}")
        self.get_logger().info(f"Angle: {angle}")
        self.get_logger().info(f"Angle error: {angle_error}")

        # Check in which direction to rotate
        # if angle_error > 0:
        #     angle_steps = -1.0
        # else:
        #     angle_steps = 1.0

        if np.abs(angle_error) > 0.05:
            self.get_logger().info(f"Rotating")
            self.twist.angular.z = 0.06 # * angle_steps
            self.publisher_twist.publish(self.twist)
        else:
            self.stop()
            self.centered = True
        return

    def move(self, waypoint):
        # Get the current position
        curr_pos = np.array(
            [self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y]
        )
        # Compute the vector between the current position and the waypoint
        # Calculate the vector
        pos_vector = waypoint - curr_pos
        # Calculate the distance to the waypoint
        dist_goal = np.linalg.norm(pos_vector)
        # Calculate the unit vector
        u_vector = pos_vector / dist_goal

        self.get_logger().info(f"Moving")
        self.get_logger().info(f"Current position: {curr_pos}")
        self.get_logger().info(f"Goal position: {waypoint}")
        self.get_logger().info(f"Distance to goal: {dist_goal}")
        self.get_logger().info(f"Unit vector: {u_vector}")

        if dist_goal > 0.3:
            self.twist.linear.x = 0.06
            # self.twist.linear.y = u_vector[1] * 0.1
            self.twist.angular.z = 0.0
            self.publisher_twist.publish(self.twist)
        else:
            self.stop()
            self.reached_goal = True
        return

    def main_node(self):
        # self.get_logger().info("Starting main node")

        # Iterate the number of times specified in the parameter n_loop (number of times the trajectory is repeated)
        if self.n_loop > 0:
            # Take off
            if self.start:
                self.start = False
                if self.sim.value:
                    self.send_request("takeoff")
                else:
                    self.take_off()

            # Iterate the number of times specified in the parameter n_loop (number of times the trajectory is repeated)
            # Check if the goal has been reached
            if not self.reached_goal:
                print(f"Waypoint {self.idx}: {self.waypoints[self.idx]}")
                if not self.centered:
                    self.rotate_goal(self.waypoints[self.idx])
                else:
                    self.move(self.waypoints[self.idx])
            else:
                self.idx += 1
                self.reached_goal = False
                self.centered = False
                if self.idx == len(self.waypoints):
                    self.stop()
                    self.n_loop -= 1
                    self.idx = 0
                    self.get_logger().info(f"Trajectory completed")
                    self.get_logger().info(f"Number of loops remaining: {self.n_loop}")
        else:
            self.get_logger().info(f"Trajectory completed")
            self.stop()
            if self.sim.value:
                self.send_request("land")
            else:
                self.land()
            self.completed = True
            # exit()


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    minimal_client = Tello()

    try:
        minimal_client.get_logger().info("Starting node")

        rclpy.spin(minimal_client)

    except KeyboardInterrupt:
        pass
    finally:
        minimal_client.get_logger().info("Shutting down")
        if minimal_client.sim.value:
            minimal_client.send_request("land")
        else:
            minimal_client.land()
        minimal_client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
