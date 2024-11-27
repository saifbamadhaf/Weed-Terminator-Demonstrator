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

# import ros2 packages
import rclpy
import random
import yaml
import os

from rclpy.node import Node
from rclpy.time import Time

from ament_index_python import get_package_share_directory

# import generic tool interface messages
from detection_interfaces.msg import Detections     
from detection_interfaces.msg import Detected     
from detection_interfaces.msg import Conveyor 

def detection_to_msg(type, x, y, z):    # Converts variables to a msg Detected() this message can then be placed in in detections message
    detection_msg = Detected()
    detection_msg.type = type
    detection_msg.position.x = x
    detection_msg.position.y = y
    detection_msg.position.z = z       
    return detection_msg 

def msg_to_detection(detection_msg):    # Detection messaged converted back to variables
    type = detection_msg.type
    x = detection_msg.position.x
    y = detection_msg.position.y 
    z = detection_msg.position.z       
    return type, x, y, z 

def print_detections(detections_msg):   # Prints the detections in terminal
    size = range(len(detections_msg.detected))

    for i in size:
        type, x, y, z = msg_to_detection(detections_msg.detected[i])
        print("-Type: ", type)
        print("Position:")
        print("     x: ", x)
        print("     y: ", y)
        print("     z: ", z)

class StaticPublisher(Node):

    def __init__(self):
        super().__init__('publisher_static')
        self.publisher_detections = self.create_publisher(Detections, 'filtered_detections', 10) # create publisher for topic /detections which is a message type detections

        self.subscriber_detection = self.create_subscription(Detections, 'detections', self.detection_callback, 10)
        self.subscriber_detection       
        
        self.get_logger().info('Static detection publisher started.')

        self.has_sent = False

    def detection_callback(self, detections_msg):       
        msg = Detections()
        
        msg.header = detections_msg.header

        for d in detections_msg.detected:
            d.position.x += 1.00

            msg.detected.append(d)
        if not self.has_sent:
            self.publisher_detections.publish(msg)
            print_detections(msg)
            self.get_logger().info('Static detection transformed and published.')
            self.has_sent = True
        #raise SystemExit

def main(args=None):
    rclpy.init(args=args)

    static_publisher = StaticPublisher()

    try:
        rclpy.spin(static_publisher)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Static detection publisher stopped.')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    static_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
