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
from roos_package.msg import Detection
from roos_package.msg import Detections as PFDetections
from roos_package.msg import Point
from roos_package.msg import XYDir

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

class PixelFarmingPublisher(Node):

    def __init__(self):
        super().__init__('publisher_detections')
        self.publisher_detections = self.create_publisher(Detections, 'detections', 10) # create publisher for topic /detections which is a message type detections
        self.publisher_motion = self.create_publisher(Conveyor, 'conveyor', 10) # create publisher for topic /detections which is a message type detections

        self.subscriber_motion = self.create_subscription(XYDir, 'roos/external/location', self.motion_callback, 10)
        self.subscriber_detection = self.create_subscription(PFDetections, 'roos/external/detections', self.detection_callback, 10)
        self.subscriber_motion
        self.subscriber_detection       

        self.pf_location_x = 0
        self.pf_location_y = 0
        
        self.get_logger().info('PixelFarming message translator started.')

    def motion_callback(self, motion_msg):       
        self.pf_location_x = motion_msg.x
        self.pf_location_y = motion_msg.y
        msg = Conveyor()
        if motion_msg.timestamp >= 0:
            t = Time(nanoseconds=motion_msg.timestamp * 1000 * 1000)
            msg.header.stamp = t.to_msg()
        msg.distance_travelled = motion_msg.y #assuming forward movement only!
        self.publisher_motion.publish(msg)
        
    def detection_callback(self, detections_msg):       
        msg = Detections()
        if detections_msg.timestamp >= 0:
            t = Time(nanoseconds=detections_msg.timestamp * 1000 * 1000)
            msg.header.stamp = t.to_msg()
        
        for detection in detections_msg.detections:
            d = Detected()
            d.type = 'weed'
            #d.position.x = -(detection.location.y - self.pf_location_y) + 0.466
            d.position.x = -detection.location.y + 0.466
            #d.position.x = detection.location.y - self.pf_location_y
            #d.position.x = (detection.location.y - self.pf_location_y - 0.466)
            d.position.y = detection.location.x - self.pf_location_x + 0.30   ##TODO: remove robot_location offset here and send coordinates in world frame. PositionController currently only compensates forward motion. Should be fixed later

            msg.detected.append(d)

        self.publisher_detections.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    pf_publisher = PixelFarmingPublisher()

    rclpy.spin(pf_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pf_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
