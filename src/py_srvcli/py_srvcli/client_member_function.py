import sys

from custom_interfaces.srv import Sensor3DOF
from custom_interfaces.msg import Sensor3DOF as SensorMsg
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        cb_group = ReentrantCallbackGroup()

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
        self.data1 = self.data2 = None

        # Initialize publisher
        self.publisher_ = self.create_publisher(SensorMsg, 'topic', 10)

        # Initialize a slow timer to call the service with the sensor data
        timer_period = 0.5  # seconds
        timer = self.create_timer(timer_period, self.call_service, callback_group=cb_group)

        # Initialize a fast timer to publish the sensor data to a topic
        timer_period2 = 0.002  # seconds
        timer2 = self.create_timer(timer_period2, self.publish_data, callback_group=cb_group)
    
    async def call_service(self):
        try:
            future = self.cli1.call_async(self.req1)
            result = await future
        finally:
            self.data1 = result
            self.get_logger().info(
                'Receved and published data for server %d' % 1)
        
        try:
            future = self.cli2.call_async(self.req2)
            result = await future
        finally:
            self.data2 = result
            self.get_logger().info(
                'Receved and published data for server %d' % 2)

    def publish_data(self):
        if self.data1:
            msg = SensorMsg()
            msg.id = 1
            msg.data.x = self.data1.x
            msg.data.y = self.data1.y
            msg.data.z = self.data1.z
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing %.0d: "%f, %f, %f"' % (msg.id, msg.data.x, msg.data.y, msg.data.z))

        if self.data2:
            msg = SensorMsg()
            msg.id = 2
            msg.data.x = self.data2.x
            msg.data.y = self.data2.y
            msg.data.z = self.data2.z
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing %.0d: "%f, %f, %f"' % (msg.id, msg.data.x, msg.data.y, msg.data.z))


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    
    rclpy.spin(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
