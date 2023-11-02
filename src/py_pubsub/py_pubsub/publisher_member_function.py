# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from custom_interfaces.msg import Sensor3DOF

# Simple test publisher, not used in solution.
class MinimalPublisher(Node):

    def __init__(self):
        # Initialize node and setup publisher
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Sensor3DOF, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0

    # Publish placeholder data to topic
    def timer_callback(self):
        msg = Sensor3DOF()
        msg.id = 1
        msg.data.x = self.i
        msg.data.y = self.i + 1
        msg.data.z = self.i + 2
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f, %f, %f, %f"' % (msg.id, msg.data.x, msg.data.y, msg.data.z))
        self.i += 1


def main(args=None):
    # Initialize node
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    # Handle callbacks
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
