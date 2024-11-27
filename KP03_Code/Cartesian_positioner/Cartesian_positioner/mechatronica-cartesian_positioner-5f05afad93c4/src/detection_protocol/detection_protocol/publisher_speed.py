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
import random
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher_speed')
        self.publisher_ = self.create_publisher(TwistStamped, 'speed', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        #msg.twist.linear.x = random.uniform(0.14, 0.16)
        msg.twist.linear.x = 0.1

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing speed')

        #timestamp = Time.from_msg(msg.header.stamp)
        #print("Timestamp in nanoseconds: ", timestamp.nanoseconds)
        #print("Speed: %f m/s" % msg.twist.linear.x )


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
