#! /home/henascen/.cache/pypoetry/virtualenvs/ar-mrs-RV7DiSQi-py3.8/bin/python
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
from tf_transformations import euler_from_quaternion

import time


class TurtleBot(Node):

    ODOM_DATA_RECORD_PATH = (
        'src/uwb_real_trajectories/test_data/20230814'
        f'/opti_as_feedback/trajectory_odom_{time.time()}'
    )
    OPTI_DATA_RECORD_PATH = (
        'src/uwb_real_trajectories/test_data/20230814'
        f'/opti_as_feedback/trajectory_opti_{time.time()}'
    )

    def __init__(self):
        # Generate a node with a random name number
        super().__init__(f"TBOT_trajectory_{randint(0, 1000)}")
        # Declare the parameters
        self.declare_parameter("n_loop", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("trajectory", rclpy.Parameter.Type.STRING)
        self.declare_parameter("sim", rclpy.Parameter.Type.BOOL)
        self.declare_parameter("domain_bridge", rclpy.Parameter.Type.BOOL)
        # Get the parameters
        self.n_loop = self.get_parameter_or("n_loop", None)
        self.trajectory = self.get_parameter_or("trajectory", None)
        self.sim = self.get_parameter_or("sim", None)
        self.domain_bridge = self.get_parameter_or("domain_bridge", False)

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
        # Create topics assuming that the current DOMAIN is for T06 (ID=26)
        if self.domain_bridge:
            self.odom_sub = self.create_subscription(
                Odometry,
                "/T02/odom",
                self.odom_callback,
                qos_profile=qos_policy
            )
            self.publisher_twist = self.create_publisher(
                Twist,
                "/T02/cmd_vel",
                1
            )
            self.mocap_sub = self.create_subscription(
                PoseStamped,
                "/vrpn_client_node/turtlebotH01/pose",
                self.mocap_callback,
                qos_profile=qos_policy,
            )

            self.ODOM_DATA_RECORD_PATH += '__T02'
            self.OPTI_DATA_RECORD_PATH += '__T02'
        else:
            self.odom_sub = self.create_subscription(
                Odometry, "/odom", self.odom_callback, qos_profile=qos_policy
            )
            # Create Twist publisher
            self.publisher_twist = self.create_publisher(Twist, "/cmd_vel", 1)
            self.mocap_sub = self.create_subscription(
                PoseStamped,
                "/vrpn_client_node/turtlebotH06/pose",
                self.mocap_callback,
                qos_profile=qos_policy,
            )

            self.ODOM_DATA_RECORD_PATH += '__T06'
            self.OPTI_DATA_RECORD_PATH += '__T06'

        self.uwb_sub = self.create_subscription(
            PoseStamped, "/uwb_tag_0719", self.uwb_callback, qos_profile=qos_policy
        )

        # Internal variables
        self.scan = LaserScan()
        self.scan_points = np.array([])
        self.odometry = Odometry()
        self.mocap = PoseStamped()
        self.uwb = PoseStamped()
        self.start = True
        self.completed = False
        self.reached_goal = False
        self.centered = False
        # Trajectories
        self.trajectory_odom = []
        self.trajectory_uwb = []
        self.trajectory_mocap = []
        self.error_dist_odom = []
        self.error_dist_uwb = []
        self.center = [0, 0]
        self.resolution = 0.08
        self.theta = 0.0

        # Velocity message
        self.twist = Twist()

        self.create_timer(0.1, self.main_node)
        self.idx = 0

        if self.trajectory.value == "square":
            self.waypoints = np.array([
                # [0.0, 0.0],
                [0.0, 2.0],
                [-2.0, 2.0],
                [-2.0, 0.0],
                [0.0, 0.0],
            ])
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
        
        self.init_odom_pose = np.array([])
        self.init_opti_pose = np.array([])

        self.odom_curr_pose = np.array([])
        self.opti_curr_pose = np.array([])

        self.odom_all_locs = []
        self.opti_all_locs = []

    def odom_callback(self, msg):
        self.odometry = msg
        orientation = msg.pose.pose.orientation
        # if self.sim.value:
        _, _, self.theta = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
        if not self.init_odom_pose.size > 0:
            self.init_odom_pose = np.array(
                [
                    self.odometry.pose.pose.position.x,
                    self.odometry.pose.pose.position.y
                ]
            )
            # self.theta = self.theta + (3 * np.pi)
        
        self.odom_curr_pose = np.array(
            [
                self.odometry.pose.pose.position.x,
                self.odometry.pose.pose.position.y
            ]
        )

    def uwb_callback(self, msg):
        self.uwb = msg

    def mocap_callback(self, msg):
        self.mocap = msg
        orientation = msg.pose.orientation
        # if not self.sim.value:
        #     _, _, self.theta = euler_from_quaternion(
        #         [orientation.x, orientation.y, orientation.z, orientation.w]
        #     )

        if not self.init_opti_pose.size > 0:
            self.init_opti_pose = np.array(
                [
                    self.mocap.pose.position.x,
                    self.mocap.pose.position.y,
                ]
            )
            self.waypoints = self.waypoints + self.init_opti_pose

        self.opti_curr_pose = np.array(
            [
                self.mocap.pose.position.x,
                self.mocap.pose.position.y,
            ]
        )

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
            [self.mocap.pose.position.x, self.mocap.pose.position.y]
        )
        # Add the waypoint destination to the current position
        # to reach the new goal (new waypoint)
        if self.init_opti_pose.size > 0:
            # curr_pos = np.abs(curr_pos - self.init_opti_pose)
            # Compute the vector between the current position and the waypoint
            # Calculate the vector
            pos_vector = waypoint - curr_pos
            # Calculate the distance to the waypoint
            dist_goal = np.linalg.norm(pos_vector)
            # Calculate the unit vector
            u_vector = pos_vector / dist_goal
            # Calculate the angle between the current position and the waypoint
            angle = np.arctan2(u_vector[0], u_vector[1])
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

            if np.abs(angle_error) > 0.02:
                self.get_logger().info(f"Rotating")
                self.twist.angular.z = 0.05  # * angle_steps
                self.publisher_twist.publish(self.twist)
            else:
                self.stop()
                self.centered = True
        return

    def move(self, waypoint):
        self.get_logger().info(f"Moving")
        # Get the current position
        curr_pos = np.array(
            [self.mocap.pose.position.x, self.mocap.pose.position.y]
        )
        # Substract the initial odom position to the current position
        # to start from a 0.0, 0.0 position
        if self.init_opti_pose.size > 0:
            # curr_pos = np.abs(curr_pos - self.init_opti_pose)
            # Compute the vector between the current position and the waypoint
            # Calculate the vector
            pos_vector = waypoint - curr_pos
            # Calculate the distance to the waypoint
            dist_goal = np.linalg.norm(pos_vector)
            # Calculate the unit vector
            u_vector = pos_vector / dist_goal

            self.get_logger().info(f"Initial opti position: {self.init_opti_pose}")
            self.get_logger().info(f"Current position: {curr_pos}")
            self.get_logger().info(f"Goal position: {waypoint}")
            self.get_logger().info(f"Distance to goal: {dist_goal}")
            self.get_logger().info(f"Unit vector: {u_vector}")

            if dist_goal > 0.1:
                self.twist.linear.x = np.linalg.norm(u_vector) * 0.15
                # self.twist.linear.y = u_vector[1] * 0.1
                self.twist.angular.z = 0.0
                self.publisher_twist.publish(self.twist)
            else:
                self.get_logger().info(f"Stopping the robot")
                self.stop()
                self.reached_goal = True
        return
    
    def save_location_data_numpy(self):
        if self.odom_curr_pose.size > 0:
            self.odom_all_locs.append(
                (self.odom_curr_pose[0], self.odom_curr_pose[1])
            )
        if self.odom_curr_pose.size > 0:
            self.opti_all_locs.append(
                (self.opti_curr_pose[0], self.opti_curr_pose[1])
            )

    def main_node(self):
        # Iterate the number of times specified in the parameter n_loop (number of times the trajectory is repeated)
        if self.n_loop > 0:
            # Iterate the number of times specified in the parameter n_loop (number of times the trajectory is repeated)
            # Check if the goal has been reached
            self.get_logger().info(
                f"Current waypoint: {self.waypoints[self.idx]}"
            )
            if not self.reached_goal:
                if not self.centered:
                    self.rotate_goal(self.waypoints[self.idx])
                else:
                    self.move(self.waypoints[self.idx])
            else:
                self.get_logger().info(f"Goal reached")
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
            self.stop()
            self.completed = True
            self.get_logger().info(f"Trajectory completed")
        
        self.get_logger().info(f"Saving location data")
        self.save_location_data_numpy()


def main(args=None):
    rclpy.init(args=args)
    cam_node = TurtleBot()
    try:
        time.sleep(5)
        cam_node.get_logger().info("Starting node")
        rclpy.spin(cam_node)
    except KeyboardInterrupt:
        cam_node.stop()
        pass
    finally:
        cam_node.get_logger().info("Writing saved data from odom and opti")
        odom_all_locs_narray = np.array(cam_node.odom_all_locs)
        opti_all_locs_narray = np.array(cam_node.opti_all_locs)
        np.save(
            cam_node.ODOM_DATA_RECORD_PATH,
            odom_all_locs_narray
        )
        np.save(
            cam_node.OPTI_DATA_RECORD_PATH,
            opti_all_locs_narray
        )
        cam_node.get_logger().info("Shutting down")
        cam_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
