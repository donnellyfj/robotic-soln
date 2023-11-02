import sys

from custom_interfaces.srv import Sensor3DOF
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        # cb_group = ReentrantCallbackGroup()
        # self.did_run = self.did_get_result = False

        # Initialize service clients
        self.cli1 = self.create_client(Sensor3DOF, 'get_sensor_data1')
        while not self.cli1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service 1 not available, waiting again...')
        self.req1 = Sensor3DOF.Request()

        self.cli2 = self.create_client(Sensor3DOF, 'get_sensor_data2')
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service 2 not available, waiting again...')
        self.req2 = Sensor3DOF.Request()

        self.i = 0 # ID to help with debugging
        # self.data1 = self.data2 = None

        # Initialize publisher
        self.publisher_ = self.create_publisher(Point, 'topic', 10)
        timer_period = 0.5  # seconds
        # timer = self.create_timer(timer_period, self.timer_callback, callback_group=cb_group)
    
    def timer_callback(self):
        self.get_logger().info('%d' % self.i)
        self.publish_data(self.data1)
        self.publish_data(self.data2)
        self.i += 1

    def send_request(self, id):
        if id == 1:
            req = self.req1
            cli = self.cli1
        else:
            req = self.req2
            cli = self.cli2

        req.i = self.i
        self.future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('%d' % id)
        # self.publish_data(self.future.result())
        self.get_logger().info(
            'Receved and published data for server %d' % id)
        if id == 1:
            self.data1 = self.future.result()
            return self.data1
        else:
            self.data2 = self.future.result()
            return self.data2
        # return self.future.result()

    def publish_data(self, response):
        self.get_logger().info('here')
        msg = Point()
        msg.x = response.x
        msg.y = response.y
        msg.z = response.z
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f, %f, %f"' % (msg.x, msg.y, msg.z))


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()

    cb_group = ReentrantCallbackGroup()
    # did_run = did_get_result = False
    # minimal_client.send_request(1)
    # minimal_client.send_request(2)


    # while rclpy.ok and not did_run:
    #     rclpy.spin_once(minimal_client)

    response1 = minimal_client.send_request(1)
    # # minimal_client.publish_data(response)
    minimal_client.get_logger().info(
        'Result of get_sensor_data: %d, %d, %d' % (response1.x, response1.y, response1.z))
    
    response2 = minimal_client.send_request(2)
    # # minimal_client.publish_data(response)
    minimal_client.get_logger().info(
        'Result of get_sensor_data: %d, %d, %d' % (response2.x, response2.y, response2.z))
    
    timer_period = 0.5  # seconds
    timer = minimal_client.create_timer(timer_period, minimal_client.timer_callback, callback_group=cb_group)
    rclpy.spin(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
