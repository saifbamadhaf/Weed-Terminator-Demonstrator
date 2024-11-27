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

# imports ros2 packages
import rclpy
import random
from rclpy.node import Node
from rclpy.time import Time

# import messages of the generic tool interface 

from detection_interfaces.msg import Detections     
   
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('times_stamp')
        self.speed = 0.0

        self.sub_detections = self.create_subscription( # create subscriber 
            Detections,
            'detections',
            self.listener_callback_detections,
            10)
        self.sub_detections # prevent unused variable warning

    def listener_callback_detections(self, Detections): # obtain detections from detections topic and prints the difference in timestamp
        #time_difference = self.get_clock().now().to_msg().nanosec -Time.from_msg(selDetections.header.stamp).nanoseconds
        print('current times stamp: ', self.get_clock().now().nanoseconds)
        print('detection time stamp: ', Time.from_msg(Detections.header.stamp).nanoseconds)
        print('time difference in seconds: ', (self.get_clock().now().nanoseconds - Time.from_msg(Detections.header.stamp).nanoseconds)/1000000000 )

def main(args=None):
    
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
