import ast
import math
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import sys
from random import randint


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
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


class AnchorTfBroadcaster(Node):
    def __init__(self):
        super().__init__(f"anchor_tf_broadcaster_{randint(0, 1000)}")
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Declare the parameters
        self.declare_parameter("robot_name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("num_antennas", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("names_antennas", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("positions_antennas", rclpy.Parameter.Type.STRING)
        # Get the parameters
        robot_name = self.get_parameter_or("robot_name", None)
        if robot_name:
            self.robot_name = robot_name.value
        num_antennas = self.get_parameter_or("num_antennas", None)
        if num_antennas:
            self.num_antennas = num_antennas.value
        names_antennas = self.get_parameter_or("names_antennas", None)
        if names_antennas:
            self.names_antennas = names_antennas.value
        pos_antennas = self.get_parameter_or("positions_antennas", None)
        if pos_antennas:
            self.pos_antennas = ast.literal_eval(pos_antennas.value)
        # Print the parameters
        # self.get_logger().info(f"robot_name: {self.robot_name}")
        # self.get_logger().info(f"num_antennas: {self.num_antennas}")
        # self.get_logger().info(f"names_antennas: {self.names_antennas}")
        # self.get_logger().info(f"pos_antennas: {self.pos_antennas}")

        # if self.num_antennas:
        #     self.transformations = [
        #         tf2_ros.TransformStamped() for _ in range(self.num_antennas)
        #     ]

        # # Initialize the transform broadcaster
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # # Subscribe to a odometry topic and call handle_turtle_pose
        # self.subscription = self.create_subscription(
        #     nav_msgs.msg.Odometry,
        #     f"/{self.robot_name}/odom",
        #     self.handle_turtle_pose,
        #     qos_profile=qos_policy,
        # )
        # # Publisher of the antenna transforms
        # self.publisher = self.create_publisher(
        #     geometry_msgs.msg.TransformStamped,
        #     f"/{self.robot_name}_antennas",
        #     qos_profile=qos_policy,
        # )
        # self.subscription  # prevent unused variable warning
        # self.odom_msg = nav_msgs.msg.Odometry()

        # Get the antennas positions
        self.antennas_x_pos = self.pos_antennas.get("x", None)
        self.antennas_y_pos = self.pos_antennas.get("y", None)
        self.antennas_z_pos = self.pos_antennas.get("z", None)

        # Initialize the transform
        self.transformation = tf2_ros.TransformStamped()
        # Initialize the transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Add the antennas to the tf broadcaster
        self.create_timer(0.1, self.generate_antennas)

    def generate_antennas(self):
        # Iterate over the antennas
        for i, (name, antenna_x, antenna_y, antenna_z) in enumerate(
            zip( 
                self.names_antennas,
                self.antennas_x_pos,
                self.antennas_y_pos,
                self.antennas_z_pos,
            )
        ):
            # print(f"anntena {i} {name} {antenna_x} {antenna_y} {antenna_z}")
            t = self.transformation
            # Read message content and assign it to
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f"world"
            t.child_frame_id = f"{self.robot_name}_{name}"
            # Turtle only exists in 2D, thus we get x and y translation based on the direction of the antenna
            t.transform.translation.x = antenna_x
            t.transform.translation.y = antenna_y
            t.transform.translation.z = antenna_z
            # Set the rotation values
            # t.transform.rotation.x = orientation.x
            # t.transform.rotation.y = orientation.y
            # t.transform.rotation.z = orientation.z
            # t.transform.rotation.w = orientation.w
            # Save the transform
            self.transformation = t
            # Send the transformation
            self.tf_broadcaster.sendTransform(t)
        return


def main(args=None):
    rclpy.init(args=args)

    node = AnchorTfBroadcaster()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
