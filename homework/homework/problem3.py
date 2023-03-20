#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, SetPen, TeleportAbsolute
from std_srvs.srv import Empty


class ServiceHw(Node):

    def __init__(self):
        super().__init__('problem_3') # super() calls the Node parent class

        # Create Clear client for default turtlesim services
        self.clear = self.create_client(Empty, '/clear')
        while not self.clear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('CLEAR_DEBUG:service not available, waiting again...')
        self.clear_req = Empty.Request()


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
        def clear_routine():
            self.clear_req
            self.future = self.clear.call_async(self.clear_req)
            rclpy.spin_until_future_complete(self, self.future)
            
        # SetPen service parameters
        self.setpen_req.r = 0
        self.setpen_req.g = 0
        self.setpen_req.b = 0
        self.setpen_req.width = 5
        self.setpen_req.off = 0
        self.future = self.setpen.call_async(self.setpen_req)
        rclpy.spin_until_future_complete(self, self.future)
        clear_routine()

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
            if i == 0:
                self.get_logger().info('i=1')
                clear_routine()
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
