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


# import ros2 package's
import rclpy
import random
from rclpy.node import Node
from rclpy.time import Time

# import message for the generic tool interface
from geometry_msgs.msg import TwistStamped
from detection_interfaces.msg import Detections     
from detection_interfaces.msg import Detected 

def detection_to_msg(type, x, y, z):            # Converts variables to a msg Detected() this message can then be placed in in detections message
    detection_msg = Detected()
    detection_msg.type = type
    detection_msg.position.x = x
    detection_msg.position.y = y
    detection_msg.position.z = z       
    return detection_msg 

def msg_to_detection(detection_msg):            # Detection messaged converted back to variables
    type = detection_msg.type
    x = detection_msg.position.x
    y = detection_msg.position.y 
    z = detection_msg.position.z       
    return type, x, y, z 

def print_detections(detections_msg):           # Prints the detections in terminal
    size = range(len(detections_msg.detected))

    for i in size:
        type, x, y, z = msg_to_detection(detections_msg.detected[i])
        print("-Type: ", type)
        print("-Position:")
        print("     x: ", x)
        print("     y: ", y)
        print("     z: ", z)
    
def detections_to_list(detections):             # Detections msg to a list of variables, this function uses the function msg_to_detection
    size = range(len(detections.detected))
    list = []
    for i in size:
        type, x, y, z = msg_to_detection(detections.detected[i])
        object = [type, x, y, z ]
        list.append(object)
    return list

def filter(self, Detections):   # Here all the incomming detections will be filtered on double detections
    # here the code for filtering double detections will go
    return Detections
   
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('filtering')
        self.speed = 0.0
        
        self.publisher = self.create_publisher(     # create publisher for topic /filtered_detections which is a message type detections
            Detections, 
            'filtered_detections', 
            10)
        
        self.sub_speed = self.create_subscription(  # create subscriber for topic /speed which has as message type TwistStamped
            TwistStamped,
            'speed',
            self.listener_callback_speed,
            10)
        self.sub_speed # prevent unused variable warning 

        self.sub_detections = self.create_subscription( # create subscriber for topic /detections_tool_frame which is a message type detections
            Detections,
            'detections_tool_frame',
            self.listener_callback_detections,
            10)
        self.sub_detections # prevent unused variable warning

    def listener_callback_speed(self, Speed):           # place the groundspeed of the agrobot in the variable speed from speed topic
        self.speed =  float(Speed.twist.linear.x)  

    def listener_callback_detections(self, Detections): # obtain detections from topic /detections_tool_frame and execute publish_filter_data function
        self.get_logger().info('Recieved detections')
        filtered_detctions = filter(self, Detections)

        self.publisher.publish(filtered_detctions)
        self.get_logger().info('Publishing filtered detections')
        

def main(args=None):
    
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
