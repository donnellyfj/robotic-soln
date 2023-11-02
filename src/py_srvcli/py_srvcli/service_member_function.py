from custom_interfaces.srv import Sensor3DOF

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv1 = self.create_service(Sensor3DOF, 'get_sensor_data1', self.get_sensor_data1_callback)
        self.srv2 = self.create_service(Sensor3DOF, 'get_sensor_data2', self.get_sensor_data2_callback)

    def get_sensor_data1_callback(self, request, response):
        # response.sum = request.a + request.b + request.c
        i = float(request.i)
        response.x = i
        response.y = i + 1
        response.z = i + 2
        self.get_logger().info('Incoming request: %d' % request.i)

        return response
    
    def get_sensor_data2_callback(self, request, response):
        # response.sum = request.a + request.b + request.c
        i = float(request.i)
        response.x = i + 3
        response.y = i + 4
        response.z = i + 5
        self.get_logger().info('Incoming request: %d' % request.i)

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
