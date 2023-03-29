import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose


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


class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        # Declare and acquire `boatname` parameter
        self.boatname = self.declare_parameter(
          'boatname', 'usv_').get_parameter_value().string_value

        # Declare and acquire `obstaclename` parameter
        self.obstaclename = self.declare_parameter(
          'obstaclename', 'obstacle_').get_parameter_value().string_value
        
        self.observername = self.declare_parameter(
          'observername', 'observer_').get_parameter_value().string_value


        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)


        # Subscribe to a turtle{1}{2}/pose topic and call handle pose functions
        # callback function on each message
        self.subscription_1 = self.create_subscription(
            Pose,
            f'/Patrick_turtlesim/{self.boatname}/pose',
            self.handle_boat_pose,
            1)
        self.subscription_1  # prevent unused variable warning

        self.subscription_2 = self.create_subscription(
            Pose,
            f'/Patrick_turtlesim/{self.obstaclename}/pose',
            self.handle_obstacle_pose,
            1)
        self.subscription_2  # prevent unused variable warning

        self.subscription_3 = self.create_subscription(
            Pose,
            f'/Patrick_turtlesim/{self.observername}/pose',
            self.handle_observer_pose,
            1)
        self.subscription_3  # prevent unused variable warning

        # create a subscription to multiple Pose topics

    def handle_boat_pose(self, msg):
        t_world = self.transform_constructor('world', 'ENU', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.tf_broadcaster.sendTransform(t_world)
        t_coord = self.transform_constructor('ENU', 'NED', 20.0, 10.0, 0.0, np.pi/2, 0.0, np.pi)
        self.tf_broadcaster.sendTransform(t_coord)
        t_boat = self.transform_constructor('ENU', self.boatname, msg.x, msg.y, 0.0, msg.theta, 0.0, 0.0)
        self.tf_broadcaster.sendTransform(t_boat)
    
    def handle_obstacle_pose(self, msg):
        # t_world = self.transform_constructor('world', 'ENU', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        # self.tf_broadcaster.sendTransform(t_world)
        # t_coord = self.transform_constructor('ENU', 'NED', 20.0, 10.0, 0.0, np.pi/2, 0.0, np.pi)
        # self.tf_broadcaster.sendTransform(t_coord)
        t_obstacle = self.transform_constructor(self.observername, self.obstaclename, msg.x, msg.y, 0.0, msg.theta, 0.0, 0.0)
        self.tf_broadcaster.sendTransform(t_obstacle)
    
    def handle_observer_pose(self, msg):
        t_world = self.transform_constructor('world', 'ENU', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.tf_broadcaster.sendTransform(t_world)
        t_coord = self.transform_constructor('ENU', 'NED', 20.0, 10.0, 0.0, np.pi/2, 0.0, np.pi)
        self.tf_broadcaster.sendTransform(t_coord)
        t_observer = self.transform_constructor('ENU', self.observername, msg.x, msg.y, 0.0, msg.theta, 0.0, 0.0)
        self.tf_broadcaster.sendTransform(t_observer)

    def transform_constructor(self, parent_frame, child_frame, x, y, z, yaw, pitch, roll):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()