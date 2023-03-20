#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, SetPen, TeleportAbsolute


class ServiceHw(Node):

    def __init__(self):
        super().__init__('problem_3') # super() calls the Node parent class

        # # Create a Spawn client for default turtlesim services
        # self.spawn = self.create_client(Spawn, 'Spawn')
        # while not self.spawn.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.spawn_req = Spawn.Request()

        # Create SetPen client for default turtlesim services
        self.setpen = self.create_client(SetPen, '/turtle1/set_pen') # srv_name is the existing service (server)
        while not self.setpen.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SETPEN_DEBUG:service not available, waiting again...')
        self.setpen_req = SetPen.Request()

        # Create TeleportAbsolute client for default turtlesim services
        self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute') # srv_name is the existing service (server)
        while not self.teleport.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('TELEPORT_DEBUG:service not available, waiting again...')
        self.teleport_req = TeleportAbsolute.Request()

    def send_request(self):
        # Spawn service parameters
        # self.spawn_req.x = 20.0
        # self.spawn_req.y = 10.0
        # self.spawn_req.theta = 0.0
        # self.spawn_req.name = 'turtle1'
        # self.future = self.cli.call_async(self.spawn_req)
        # rclpy.spin_until_future_complete(self, self.future)

        # SetPen service parameters
        self.setpen_req.r = 0
        self.setpen_req.g = 0
        self.setpen_req.b = 0
        self.setpen_req.width = 5
        self.setpen_req.off = 0
        self.future = self.setpen.call_async(self.setpen_req)
        rclpy.spin_until_future_complete(self, self.future)

        # TeleportAbsolute_1 service parameters 
        self.teleport_req.x = 20.0
        self.teleport_req.y = 15.0
        self.teleport_req.theta = 0.0
        self.future = self.teleport.call_async(self.teleport_req)
        rclpy.spin_until_future_complete(self, self.future)

        # TeleportAbsolute_2 service parameters 
        self.teleport_req.x = 20.0
        self.teleport_req.y = 5.0
        self.teleport_req.theta = 0.0
        self.future = self.teleport.call_async(self.teleport_req)
        rclpy.spin_until_future_complete(self, self.future)

        # TeleportAbsolute_3 service parameters 
        self.teleport_req.x = 20.0
        self.teleport_req.y = 10.0
        self.teleport_req.theta = 0.0
        self.future = self.teleport.call_async(self.teleport_req)
        rclpy.spin_until_future_complete(self, self.future)

        # TeleportAbsolute_4 service parameters 
        self.teleport_req.x = 15.0
        self.teleport_req.y = 10.0
        self.teleport_req.theta = 0.0
        self.future = self.teleport.call_async(self.teleport_req)
        rclpy.spin_until_future_complete(self, self.future)

        # TeleportAbsolute_5 service parameters 
        self.teleport_req.x = 25.0
        self.teleport_req.y = 10.0
        self.teleport_req.theta = 0.0
        self.future = self.teleport.call_async(self.teleport_req)
        rclpy.spin_until_future_complete(self, self.future)


def main(args=None):
    rclpy.init()
    problem3 = ServiceHw()
    problem3.send_request()
    # problem3.get_logger().info()
    problem3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
