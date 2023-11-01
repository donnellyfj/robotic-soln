from custom_interfaces.srv import Sensor3DOF

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Sensor3DOF, 'get_sensor_data', self.get_sensor_data_callback)

    def get_sensor_data_callback(self, request, response):
        # response.sum = request.a + request.b + request.c
        i = float(request.i)
        response.x = i
        response.y = i + 1
        response.z = i + 2
        self.get_logger().info('Incoming request: %d' % request.i)

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
