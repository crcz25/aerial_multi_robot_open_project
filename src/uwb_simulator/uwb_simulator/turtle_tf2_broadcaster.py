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

class AntennaTfBroadcaster(Node):

    def __init__(self):
        super().__init__(f'turtle_tf_broadcaster_{randint(0, 1000)}')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # Get parameters from the parameter server
        # if len(sys.argv) > 1:
        #     self.robot_name = sys.argv[1]
        #     self.num_antennas = int(sys.argv[2])
        #     self.names_antennas = sys.argv[3]
        #     self.get_logger().info(f"robot_name: {self.robot_name}")
        #     self.get_logger().info(f"num_antennas: {self.num_antennas}")
        #     self.get_logger().info(f"names_antennas: {self.names_antennas}")
        #     self.transformations = [tf2_ros.TransformStamped() for _ in range(self.num_antennas)]
        # Declare the parameters
        self.declare_parameter('robot_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('num_antennas', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter(
            'names_antennas',
            rclpy.Parameter.Type.STRING_ARRAY
        )
        # Get the parameters
        robot_name = self.get_parameter_or('robot_name', None)
        if robot_name:
            self.robot_name = robot_name.value
        num_antennas = self.get_parameter_or('num_antennas', None)
        if num_antennas:
            self.num_antennas = num_antennas.value
        names_antennas = self.get_parameter_or('names_antennas', None)
        if names_antennas:
            self.names_antennas = names_antennas.value
        # Print the parameters
        self.get_logger().info(f"robot_name: {self.robot_name}")
        self.get_logger().info(f"num_antennas: {self.num_antennas}")
        self.get_logger().info(f"names_antennas: {self.names_antennas}")

        if self.num_antennas:
            self.transformations = [
                tf2_ros.TransformStamped()
                for _ in range(self.num_antennas)
            ]

        # Initialize the transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Subscribe to a odometry topic and call handle_turtle_pose
        self.subscription = self.create_subscription(nav_msgs.msg.Odometry,
                                                     f'/{self.robot_name}/odom',
                                                     self.handle_turtle_pose,
                                                     qos_profile=qos_policy)
        # Publisher of the antenna transforms
        self.publisher = self.create_publisher(geometry_msgs.msg.TransformStamped, f'/{self.robot_name}_antennas', qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning
        self.odom_msg = nav_msgs.msg.Odometry()

    def publish_transform(self):
        pass

    def generate_antennas(self):
        # Get the turtle pose
        pose = self.odom_msg.pose.pose
        # Get the turtle orientation
        orientation = self.odom_msg.pose.pose.orientation

        # Generate the posible directions based on the number of antennas
        directions = np.linspace(0, 2*np.pi, self.num_antennas, endpoint=False)

        # Iterate over the antennas
        for i in range(self.num_antennas):
            # Create a transform message and set its values
            t = self.transformations[i]
            # Read message content and assign it to
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f"{self.robot_name}_base_link"
            t.child_frame_id = f"{self.robot_name}_{self.names_antennas[i]}"
            # Turtle only exists in 2D, thus we get x and y translation based on the direction of the antenna
            t.transform.translation.x = 0.5*np.cos(directions[i])# + pose.position.x
            t.transform.translation.y = 0.5*np.sin(directions[i])# + pose.position.y
            t.transform.translation.z = pose.position.z
            # Set the rotation values
            t.transform.rotation.x = orientation.x
            t.transform.rotation.y = orientation.y
            t.transform.rotation.z = orientation.z
            t.transform.rotation.w = orientation.w
            # Save the transform
            self.transformations[i] = t
            # Send the transformation
            self.tf_broadcaster.sendTransform(t)
            # Add the position of the robot to have the global position of the antenna
            t.transform.translation.x += pose.position.x
            t.transform.translation.y += pose.position.y
            t.transform.translation.z = pose.position.z
            # Publish the transformation
            self.publisher.publish(t)
            

    def handle_turtle_pose(self, msg):
        #print("handle_turtle_pose")
        self.odom_msg = msg
        self.generate_antennas()

    # def publish_transform(self):
    #     # Set the header timestamp to the current time
    #     transform = self.tf_buffer.lookup_transform(self.parent_frame_id, self.antenna_frame_id, rclpy.time.Time())

    #     transform.header.stamp = self.get_clock().now().to_msg()

    #     # Publish the transform
    #     self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    print(args)

    node = AntennaTfBroadcaster()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()