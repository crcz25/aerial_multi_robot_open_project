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

class TfBroadcaster(Node):

    def __init__(self):
        super().__init__(f'turtle_tf2_base_rename_{randint(0, 1000)}')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # Get parameters from the parameter server
        if len(sys.argv) > 1:
            self.robot_name = sys.argv[1]
            self.get_logger().info(f"robot_name: {self.robot_name}")
        self.transformation = tf2_ros.TransformStamped()

        # Initialize the transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Subscribe to a odometry topic and call handle_turtle_pose
        self.subscription = self.create_subscription(nav_msgs.msg.Odometry,
                                                     f'/{self.robot_name}/odom',
                                                     self.handle_turtle_pose,
                                                     qos_profile=qos_policy)
        self.odom_msg = nav_msgs.msg.Odometry()

    def rename_base_footprint(self):
        # Get the turtle pose
        pose = self.odom_msg.pose.pose
        # Get the turtle orientation
        orientation = self.odom_msg.pose.pose.orientation

        t = self.transformation
        # Read message content and assign it to
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f"odom"
        t.child_frame_id = f"base_footprint_{self.robot_name}"
        # Turtle only exists in 2D, thus we get x and y translation based on the direction of the antenna
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        # Set the rotation values
        t.transform.rotation.x = orientation.x
        t.transform.rotation.y = orientation.y
        t.transform.rotation.z = orientation.z
        t.transform.rotation.w = orientation.w
        # Save the transform
        self.transformation = t
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def handle_turtle_pose(self, msg):
        #print("handle_turtle_pose")
        self.odom_msg = msg
        self.rename_base_footprint()

def main(args=None):
    rclpy.init(args=args)
    print(args)

    node = TfBroadcaster()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()