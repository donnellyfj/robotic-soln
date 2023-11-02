from custom_interfaces.srv import Sensor3DOF

import rclpy
from rclpy.node import Node
import socket
import numpy as np

# Create TCP/IP sockets
sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the sockets to the ports where the servers are listening
server_address1 = ('127.0.0.3', 10000)
print('connecting to {} port {}'.format(*server_address1))
sock1.connect(server_address1)

server_address2 = ('127.0.0.1', 10000)
print('connecting to {} port {}'.format(*server_address2))
sock2.connect(server_address2)

# Minimal Service
#   This node acts as both a service that returns the most recent data from
#   a sensor when queried by a client. The node mantains constant connection
#   with two sensors and uses a custom interface to handle the passing of 3
#   DOF data. The node will halt operation if connection to a sensor is lost.
class MinimalService(Node):

    def __init__(self):
        # Initialize node from superclass and create a service for each sensor.s
        super().__init__('minimal_service')
        self.srv1 = self.create_service(Sensor3DOF, 'get_sensor_data1', self.get_sensor_data1_callback)
        self.srv2 = self.create_service(Sensor3DOF, 'get_sensor_data2', self.get_sensor_data2_callback)

        # Request one sample from the sensors at a time
        # NOTE: The system is currently set up as a feed such that at the time of calling the
        #       service, the most recent sensor position is returned. Since the rate of the
        #       sensor sampling and the delay from the overhead of the calls are much faster
        #       than the rate of the service client and publisher calls, it is not feasible
        #       to stream every data point, and thus we can simply sample the most recent
        #       sensor value.
        #       If is advantageous to publish the data to the topic in batches as opposed to a
        #       steady stream, we can simply rework the solution to return the data in array
        #       form, allowing us to capture all samples retroactively. The number of samples
        #       that should be used in this case would be:
        #       numSamples = (sensorRate(Hz)) * (delay(s) + MAX(serviceRate(s), publishRate(s)))
        #       This value is the amount of data points a sensor produces in between calls, as
        #       each call takes delay + MAX(serviceRate, publishRate) seconds, and each sensor
        #       producess sensorRate values per second.
        self.number_of_samples = 1

    # Return most recent data from sensor 1
    def get_sensor_data1_callback(self, request, response):
        self.get_logger().info('Incoming request: %d' % request.i)

        # Request 1 sample from the sensor
        print("Requesting samples...")
        message_string = str(self.number_of_samples)
        message = message_string.encode()
        sock1.sendall(message)
        print("Message sent")

        # Receive data from sensor
        byte_data = sock1.recv(10000)
        data =  np.frombuffer(byte_data)
        print("Data received!")
        print(data[0], data[1], data[2])
        response.x = data[0]
        response.y = data[1]
        response.z = data[2]

        return response

    # Return most recent data from sensor 2
    def get_sensor_data2_callback(self, request, response):
        self.get_logger().info('Incoming request: %d' % request.i)

        # Request 1 sample from the sensor
        print("Requesting samples...")
        message_string = str(self.number_of_samples)
        message = message_string.encode()
        sock2.sendall(message)
        print("Message sent")

        # Receive data from sensor
        byte_data = sock2.recv(10000)
        data =  np.frombuffer(byte_data)
        print("Data received!")
        print(data[0], data[1], data[2])
        response.x = data[0]
        response.y = data[1]
        response.z = data[2]

        return response


def main():
    # Initilaize ROS node
    rclpy.init()
    minimal_service = MinimalService()

    # Handle callbacks
    rclpy.spin(minimal_service)

    # Clean up on shutdown
    rclpy.shutdown()

    # Clean up the connection
    print('closing socket')
    sock1.close()
    sock2.close()


if __name__ == '__main__':
    main()
