import math
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg

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
        super().__init__('antenna_tf_broadcaster')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        # Initialize the transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        # Subscribe to a odometry topic and call handle_turtle_pose
        # callback function on each message
        # self.subscription = self.create_subscription(nav_msgs.Odometry,f'/{self.turtlename}/pose',self.handle_turtle_pose,1)
        self.subscription = self.create_subscription(nav_msgs.msg.Odometry, f'/odom', self.handle_turtle_pose, qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning
        self.odom_msg = nav_msgs.msg.Odometry()

        # # Define the name of the base frame of the TurtleBot and the name of the antenna frame
        # self.parent_frame_id = 'odom'
        # self.antenna_frame_id = 'antenna'

        # # Create the tf buffer and tf broadcaster
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # # Create a transform message and set its values
        # transform = geometry_msgs.msg.TransformStamped()
        # transform.header.frame_id = self.parent_frame_id
        # transform.child_frame_id = self.antenna_frame_id
        # transform.transform.translation.x = 1.0  # Set the translation values
        # transform.transform.translation.y = 0.0
        # transform.transform.translation.z = 0.0
        # # transform.transform.rotation.x = 0.0  # Set the rotation values
        # # transform.transform.rotation.y = 0.0
        # # transform.transform.rotation.z = 0.0
        # # transform.transform.rotation.w = 1.0

        # # Publish the transform once per second
        # self.timer = self.create_timer(1.0, self.publish_transform)

    def publish_transform(self):
        pass

    def generate_antennas(self, **kwargs):
        # Retrieve the number of antennas and the frame names from the kwargs
        n_antennas = kwargs.get('n_antennas', 4)
        antenna_frame_id = kwargs.get('antenna_frame_id', 'antenna')
        parent_frame_id = kwargs.get('parent_frame_id', 'base_footprint')
        # Get the turtle pose
        pose = kwargs.get('pose', self.odom_msg.pose.pose)
        # Get the turtle orientation
        orientation = kwargs.get('orientation', self.odom_msg.pose.pose.orientation)

        # Generate the posible directions based on the number of antennas
        directions = np.linspace(0, 2*np.pi, n_antennas, endpoint=False)

        for i in range(n_antennas):
            # Create a transform message and set its values
            t = tf2_ros.TransformStamped()
            # Read message content and assign it to
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent_frame_id
            t.child_frame_id = f"{antenna_frame_id}_{i}"
            # Turtle only exists in 2D, thus we get x and y translation based on the direction of the antenna
            t.transform.translation.x = 0.5*np.cos(directions[i])
            t.transform.translation.y = 0.5*np.sin(directions[i])
            t.transform.translation.z = 0.0
            # Set the rotation values
            t.transform.rotation.x = orientation.x
            t.transform.rotation.y = orientation.y
            t.transform.rotation.z = orientation.z
            t.transform.rotation.w = orientation.w

            print(f"t: {t}")

            self.tf_broadcaster.sendTransform(t)

    def handle_turtle_pose(self, msg):
        print("handle_turtle_pose")
        self.odom_msg = msg
        pose = msg.pose.pose
        orientation = msg.pose.pose.orientation
        self.generate_antennas(n_antennas=4, antenna_frame_id='antenna', parent_frame_id='base_footprint', pose=pose, orientation=orientation)
        # self.generate_antennas(n_antennas=3, antenna_frame_id='antenna', parent_frame_id='odom', pose=pose, orientation=orientation)

        # # Create a transform message and set its values
        # t = tf2_ros.TransformStamped()

        # # Read message content and assign it to
        # # corresponding tf variables
        # t.header.stamp = self.get_clock().now().to_msg()
        # # t.header.frame_id = 'odom'
        # t.header.frame_id = 'base_footprint'
        # t.child_frame_id = f"antenna_{msg.header.frame_id}_1"

        # # Turtle only exists in 2D, thus we get x and y translation
        # # coordinates from the message and set the z coordinate to 0
        # t.transform.translation.x = pose.position.x + 0.5
        # t.transform.translation.y = pose.position.y
        # t.transform.translation.z = 0.0

        # # For the same reason, turtle can only rotate around one axis
        # # and this why we set rotation in x and y to 0 and obtain
        # # rotation in z axis from the message
        # #  q = quaternion_from_euler(0, 0, msg.theta)
        # q = [orientation.x, orientation.y, orientation.z, orientation.w]
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]

        # print(f"\nt: {t}")

        # Send the transformation
        # self.tf_broadcaster.sendTransform(t)

    # def publish_transform(self):
    #     # Set the header timestamp to the current time
    #     transform = self.tf_buffer.lookup_transform(self.parent_frame_id, self.antenna_frame_id, rclpy.time.Time())

    #     transform.header.stamp = self.get_clock().now().to_msg()

    #     # Publish the transform
    #     self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)

    node = AntennaTfBroadcaster()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()