#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Kill, Spawn, SetPen, TeleportAbsolute
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from math import pi, atan2


class ServiceHw(Node):

    def __init__(self):
        # super() calls the Node parent class
        super().__init__('problem_3') 

        # create Clear client for default turtlesim services
        self.clear = self.create_client(Empty, '/Patrick_turtlesim/clear')
        while not self.clear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('CLEAR_DEBUG:service not available, waiting again...')
        self.clear_req = Empty.Request()

        # create SetPen client for default turtlesim services
        self.setpen = self.create_client(SetPen, '/Patrick_turtlesim/turtle1/set_pen') # srv_name is the existing service (server)
        while not self.setpen.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SETPEN_DEBUG:service not available, waiting again...')
        self.setpen_req = SetPen.Request()

        # create TeleportAbsolute client for default turtlesim services
        self.teleport = self.create_client(TeleportAbsolute, '/Patrick_turtlesim/turtle1/teleport_absolute') # srv_name is the existing service (server)
        while not self.teleport.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('TELEPORT_DEBUG:service not available, waiting again...')
        self.teleport_req = TeleportAbsolute.Request()

        # create Spawn client for default turtlesim services
        self.spawn = self.create_client(Spawn, '/Patrick_turtlesim/spawn') # srv_name is the existing service (server)
        while not self.spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SPAWN_DEBUG:service not available, waiting again...')
        self.spawn_req = Spawn.Request()

        # create Kill client for  turtlesim services
        self.kill = self.create_client(Kill, '/Patrick_turtlesim/kill') # srv_name is the existing service (server)
        while not self.kill.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('KILL_DEBUG:service not available, waiting again...')
        self.kill_req = Kill.Request()
        
        # attributes to be accessed later
        self.usv_x = None
        self.usv_y = None

    def send_request(self):
        # all services are called in order
        # SetPen service parameters for black
        self.setpen_req.r = 0
        self.setpen_req.g = 0
        self.setpen_req.b = 0
        self.setpen_req.width = 5
        self.setpen_req.off = 0
        self.future = self.setpen.call_async(self.setpen_req)
        rclpy.spin_until_future_complete(self, self.future)

        # spawning parameters are in absolute coordinates unless stated otherwise
        # draw the coordinate axes
        coordinates = [(20.0, 10.0),
                        (20.0, 15.0),
                        (20.0, 5.0),
                        (20.0, 10.0),
                        (15.0, 10.0),
                        (25.0, 10.0)]
        for i, (x, y) in enumerate(coordinates):
            self.teleport_req.x = x
            self.teleport_req.y = y
            self.teleport_req.theta = 0.0
            self.future = self.teleport.call_async(self.teleport_req)
            rclpy.spin_until_future_complete(self, self.future)    
            if i == 0:
                self.future = self.clear.call_async(self.clear_req)
                rclpy.spin_until_future_complete(self, self.future)

        # remove drawing turtle
        self.kill_req.name = 'turtle1'
        self.future = self.kill.call_async(self.kill_req)
        rclpy.spin_until_future_complete(self, self.future)
        
        self.spawn_req.x = 20.0
        self.spawn_req.y = 10.0
        self.spawn_req.theta = 0.0
        self.spawn_req.name = 'usv'
        self.future = self.spawn.call_async(self.spawn_req)
        rclpy.spin_until_future_complete(self, self.future)

        # create subscriber for usv pose flexibility
        self.subscriber_node = rclpy.create_node('usv_checker')
        self.subscription = self.subscriber_node.create_subscription(
                Pose,
                '/Patrick_turtlesim/usv/pose',
                self.sub_callback,
                10)
        self.subscription

        # waiting function so data exists before continuing
        while self.usv_x == None:
            rclpy.spin_once(self.subscriber_node, timeout_sec=1.0)
        
        # obstacle parameters and calculations
        abs_obstacle_x = 25.0
        abs_obstacle_y = 15.0
        rel_obstacle_x = abs_obstacle_x - self.usv_x
        rel_obstacle_y = abs_obstacle_y - self.usv_y
        vector_angle = -(pi - atan2(rel_obstacle_y, rel_obstacle_x))
        spawn_parameters = [(25.0, 15.0, vector_angle, 'obstacle'),
                            (5.0, 5.0, pi/2, 'observer')]

        # spawn the obstacle and observer
        for turtles in spawn_parameters:
            self.spawn_req.x = turtles[0]
            self.spawn_req.y = turtles[1]
            self.spawn_req.theta = turtles[2]
            self.spawn_req.name = turtles[3]
            self.future = self.spawn.call_async(self.spawn_req)
            rclpy.spin_until_future_complete(self, self.future)

    # callback function to update attributes
    def sub_callback(self, msg):
        # absolute coordinates from usv spawn
        self.usv_x = msg.x
        self.usv_y = msg.y

def main(args=None):
    rclpy.init()
    problem3 = ServiceHw()
    problem3.send_request()
    problem3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
