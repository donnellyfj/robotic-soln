from custom_interfaces.srv import Sensor3DOF

import rclpy
from rclpy.node import Node
import socket
import numpy as np

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('127.0.0.3', 10000)
print('connecting to {} port {}'.format(*server_address))
sock.connect(server_address)

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv1 = self.create_service(Sensor3DOF, 'get_sensor_data1', self.get_sensor_data1_callback)
        self.srv2 = self.create_service(Sensor3DOF, 'get_sensor_data2', self.get_sensor_data2_callback)
        self.number_of_samples = 1

    def get_sensor_data1_callback(self, request, response):
        # response.sum = request.a + request.b + request.c
        # i = float(request.i)
        # response.x = i
        # response.y = i + 1
        # response.z = i + 2
        self.get_logger().info('Incoming request: %d' % request.i)
        # Request 10 samples from the sensor
        print("Requesting samples...")
        message_string = str(self.number_of_samples)
        message = message_string.encode()
        sock.sendall(message)
        print("Message sent")

        byte_data = sock.recv(10000)
        data =  np.frombuffer(byte_data)
        print("Data received!")
        print(data[0], data[1], data[2])
        response.x = data[0]
        response.y = data[1]
        response.z = data[2]

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

    # Clean up the connection
    print('closing socket')
    sock.close()


if __name__ == '__main__':
    main()
