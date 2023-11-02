import sys

from custom_interfaces.srv import Sensor3DOF
from custom_interfaces.msg import Sensor3DOF as SensorMsg
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

# Minimal Client Async
#   This node acts as both a service client and a topic publisher.
#   On a timer, the client will make a request to the service for
#   sensor data. A custom interface is use to handle passing 3 DOF
#   data from the sensors. Upon receiving a response, the data is
#   published to a topic. The most recent data retrieved by the client
#   is stored persistently, allowing the data to be published at a
#   faster rate to the topic than it is received from the service.
class MinimalClientAsync(Node):

    def __init__(self):
        # Initialize node from Node superclass
        super().__init__('minimal_client_async')

        # Allow for parallel callback execution to prevent deadlocks
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

        # Store most recent value for each sensor
        self.data1 = self.data2 = None

        # Initialize publisher
        self.publisher_ = self.create_publisher(SensorMsg, 'topic', 10)

        # Initialize a slow timer to call the service with the sensor data
        timer_period = 0.01  # seconds
        timer1 = self.create_timer(timer_period, self.call_service1, callback_group=cb_group)
        timer2 = self.create_timer(timer_period, self.call_service2, callback_group=cb_group)

        # Initialize fast timers to publish the sensor data to a topic
        timer_period2 = 0.002  # seconds
        timer3 = self.create_timer(timer_period2, self.publish_data1, callback_group=cb_group)
        timer4 = self.create_timer(timer_period2, self.publish_data2, callback_group=cb_group)
    
    # Request data from sensor 1
    async def call_service1(self):
        try:
            # Request data
            future = self.cli1.call_async(self.req1)
            result = await future
        finally:
            # Store result if successful
            self.data1 = result
            self.get_logger().info(
                'Receved data from server %d' % 1)
    
    # Request data from sensor 2
    async def call_service2(self):
        try:
            # Request data
            future = self.cli2.call_async(self.req2)
            result = await future
        finally:
            # Store result if successful
            self.data2 = result
            self.get_logger().info(
                'Receved data from server %d' % 2)

    # Publish data from sensor 1 to topic if data is available
    def publish_data1(self):
        if self.data1:
            msg = SensorMsg()
            msg.id = 1
            msg.data.x = self.data1.x
            msg.data.y = self.data1.y
            msg.data.z = self.data1.z
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing %.0d: "%f, %f, %f"' % (msg.id, msg.data.x, msg.data.y, msg.data.z))

    # Publish data from sensor 2 to topic if data is available
    def publish_data2(self):
        if self.data2:
            msg = SensorMsg()
            msg.id = 2
            msg.data.x = self.data2.x
            msg.data.y = self.data2.y
            msg.data.z = self.data2.z
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing %.0d: "%f, %f, %f"' % (msg.id, msg.data.x, msg.data.y, msg.data.z))


def main():
    # Initilaize ROS node
    rclpy.init()
    minimal_client = MinimalClientAsync()
    
    # Handle callbacks
    rclpy.spin(minimal_client)

    # Clean up on shutdown
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
