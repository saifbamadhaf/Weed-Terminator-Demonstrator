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

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher_detections')
        self.publisher_ = self.create_publisher(Detections, 'detections', 10) # create publisher for topic /detections which is a message type detections
        #timer_period = 10  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.declare_parameter('mode', 'dynamic')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        
        package_path = get_package_share_directory('detection_protocol')

        if (self.mode == "measured"):
            absolute_file_path = os.path.join(package_path, 'config/measured_positions.yaml')

        if (self.mode == "static"):
            absolute_file_path = os.path.join(package_path, 'config/static_positions.yaml')

        if (self.mode == "dynamic"):
            absolute_file_path = os.path.join(package_path, 'config/dynamic_positions.yaml')

        
        self.declare_parameter('filename', absolute_file_path)

        if (self.mode != "measured"):
            self.subscriber = self.create_subscription(Conveyor, 'conveyor', self.conveyor_callback, 10)
            self.subscriber
        self.last_turn_value_received = 0
        
        
        filename = self.get_parameter('filename').get_parameter_value().string_value

        self.declare_parameter('use_measured_z', False)
        self.use_z = self.get_parameter('use_measured_z').get_parameter_value().bool_value
        
        self.declare_parameter('z_position_ground', -0.14)
        self.z_position_ground = self.get_parameter('z_position_ground').get_parameter_value().double_value
        
        print("Z position: ", self.z_position_ground)

        file = open(filename, 'r')
        data = yaml.safe_load(file)
        self.detection_list = data['detections']
        print(self.detection_list)

        if (self.mode == "measured"): # only send once! TODO implement proper timer without conveyor simulation
            msg = Conveyor()
            msg.turns = 1
            msg.header.stamp = self.get_clock().now().to_msg()
            self.conveyor_callback(msg)

    def conveyor_callback(self, conveyor_msg):
        if (conveyor_msg.turns == self.last_turn_value_received):
            return
        
        self.last_turn_value_received = conveyor_msg.turns
        msg = Detections()
        #detections = range(4)
        #timestamp = self.get_clock().now().nanoseconds - 500000000  # "delays" the detections now it looks like a detection takes 0.5 seconds thats why the timestamp goes back

        #seconds = int(timestamp/1000000000)  
        #nanosec = timestamp - (seconds*1000000000)
        msg.header = conveyor_msg.header
        
        #msg.header.stamp._sec = seconds
        #msg.header.stamp.nanosec = nanosec
        
        #TODO publish in batches with size of positioner frame? or let positioner select reachable crops after moving_back
        zero_line_offset = -1.755 #distance between positioner zero positioner and belt zero position (publish all crops and weeds on belts only once per belt cycle)
        #zero_line_offset = -1.698 #distance between positioner zero positioner and belt zero position (publish all crops and weeds on belts only once per belt cycle)
        belt_length = 3.545 #distance between two triggered publications
        list_det = ["weed", "crop"]

        dynamic_offset = (conveyor_msg.turns-1)*belt_length
        if (self.mode == "static"):
            zero_line_offset = 1.255
            dynamic_offset = 0

        z_position = self.z_position_ground
        for i in range(len(self.detection_list)):
        #     msg.detected.append(detection_to_msg(random.choice(list_det),
        #                                 random.uniform(-0.5, 0),
        #                                 random.uniform(0, 0.50),
        #                                 random.uniform(0, 0.40)))
            if self.use_z:
                z_position = self.detection_list[i][3]
            if (self.mode == "measured"): #use measured data 'as is' without any conversions or offsets
                msg.detected.append(detection_to_msg(self.detection_list[i][0], 
                                                self.detection_list[i][1], 
                                                self.detection_list[i][2], 
                                                z_position))
            else:
                msg.detected.append(detection_to_msg(self.detection_list[i][0], 
                                                zero_line_offset-1*self.detection_list[i][1]-dynamic_offset, 
                                                self.detection_list[i][2], 
                                                z_position))

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing detections ')
        #timestamp = Time.from_msg(msg.header.stamp)
        #print("Timestamp in nanoseconds: ", timestamp.nanoseconds)
        #print_detections(msg)


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
