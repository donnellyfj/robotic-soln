import sys

from custom_interfaces.srv import Sensor3DOF
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        # Initialize service client
        self.cli = self.create_client(Sensor3DOF, 'get_sensor_data')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Sensor3DOF.Request()
        self.i = 0 # ID to help with debugging

        # Initialize publisher
        self.publisher_ = self.create_publisher(Point, 'topic', 10)

    def send_request(self):
        # self.req.x = x
        # self.req.y = y
        # self.req.z = z
        self.req.i = self.i
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def publish_data(self, response):
        msg = Point()
        msg.x = response.x
        msg.y = response.y
        msg.z = response.z
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f, %f, %f"' % (msg.x, msg.y, msg.z))
        self.i += 1


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    minimal_client.publish_data(response)
    minimal_client.get_logger().info(
        'Result of get_sensor_data: %d, %d, %d' % (response.x, response.y, response.z))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
