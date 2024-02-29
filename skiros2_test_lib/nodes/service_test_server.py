#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from time import sleep

class TestServiceServer(Node):

    def __init__(self):
        super().__init__('set_bool')
        self.srv = self.create_service(SetBool, 'set_bool', self.set_bool_callback)

    def set_bool_callback(self, request: SetBool.Request, response: SetBool.Response):
        self.get_logger().info('Incoming request\nbool: %d' % (request.data))
        sleep(4.0)
        self.get_logger().info('Request completed')

        response.success = True
        response.message = "Got %s" % request.data
        return response


def main(args=None):
    rclpy.init(args=args)

    test_service = TestServiceServer()

    rclpy.spin(test_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()