# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import math


from geometry_msgs.msg import TransformStamped
import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from rclpy.time import Time
from tf2_ros import TransformBroadcaster

from tf2_ros import TransformException, LookupTransform
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

 # import the messages of generic tool interface
from geometry_msgs.msg import TwistStamped
from detection_interfaces.msg import Detections     
from detection_interfaces.msg import Detected 

def msg_to_detection(detection_msg):            # Detection messaged converted back to variables
    type = detection_msg.type
    x = detection_msg.position.x
    y = detection_msg.position.y 
    z = detection_msg.position.z       
    return type, x, y, z 

def detections_to_list(detections):             # Detections msg to a list of variables, this function uses the function msg_to_detection
    size = range(len(detections.detected))
    list = []
    for i in size:
        type, x, y, z = msg_to_detection(detections.detected[i])
        object = [type, x, y, z ]

        list.append(object)
    return list

def detection_to_msg(type, x, y, z):            # x y z coordinates are made ready to be published as msg
    detection_msg = Detected()
    detection_msg.type = type
    detection_msg.position.x = x
    detection_msg.position.y = y
    detection_msg.position.z = z       
    return detection_msg 

def print_detections(detections_msg):           # Prints detections on command line good for debugging
    size = range(len(detections_msg.detected))

    for i in size:
        type, x, y, z = msg_to_detection(detections_msg.detected[i])
        print("-Type: ", type)
        print("Position:")
        print("     x: ", x)
        print("     y: ", y)
        print("     z: ", z)

def closest(lst, K):
     
    return lst[min(range(len(lst)), key = lambda i: abs(lst[i][0]-K))]  # returns the value with te minimum difference

def distance(self):     # calculates the distance per given speed
    dt = 0
    vel = 0
    
    le = len(self.stamp_vel_s)

    if le > 0 :                             #calculates the traveled distance. dt times speed. the frist time a speed is provided this stemp will be ignored
        vel = self.stamp_vel_s[le-1][1]
        t = self.stamp_vel_s[le-1][0]
        dt = (Time.from_msg(self.speed.header.stamp).nanoseconds - t)/1000000000
    
    self.stamp_vel_s.append([Time.from_msg(self.speed.header.stamp).nanoseconds, float(self.speed.twist.linear.x), (vel *dt)])  #saves a list of timestamp, speed and distance
    
    size = len(self.stamp_vel_s)
    if size > 30 :                 #<--- change this number to your needs. a higher number is requiered when the speed gets sent at a higher rate. 
                                    #And when the detections are sent with a lowrate in between
           self.stamp_vel_s.pop(0)  # removes the first digit to keep the list at a certain lenght
    
    # size = range(len(self.stamp_vel_s))
    # for i in size:
    #      print('time: ', self.stamp_vel_s[i][0], ' speed: ', self.stamp_vel_s[i][1], ' distance: ', self.stamp_vel_s[i][2])
    # print('----------------------------------------------------------------')

def get_transform(self, from_frame_rel, to_frame_rel):  # Read out the transform from tf tree
    try:
        when = self.get_clock().now() - rclpy.time.Duration(seconds=5.0)
        t = self.tf_buffer.lookup_transform_full(
                    target_frame=to_frame_rel,
                    target_time=rclpy.time.Time(),      #rclpy.time.Time()
                    source_frame=from_frame_rel,
                    source_time=when,
                    fixed_frame='world',
                    timeout=rclpy.duration.Duration(seconds=0.05))

        return t
    except TransformException as ex:
        self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        return
    except (LookupException, ConnectivityException, ExtrapolationException):
        self.get_logger().info('transform not ready')
        raise
        return 

def traveled_distance(self, given_start_time, given_end_time):                    #calculates the traveled distance
    
    start_time = closest(self.stamp_vel_s, given_start_time)    #Time.from_msg(self.Detections.header.stamp).nanoseconds

    start_element = self.stamp_vel_s.index(start_time)
    
    end_time = closest(self.stamp_vel_s, given_end_time)
    
    end_element = self.stamp_vel_s.index(end_time)
    
    range_d = range((start_element+1), (end_element+1))

    distance = 0
    try:
        for a in range_d:

            distance = distance + self.stamp_vel_s[a][2]
        print(self.stamp_vel_s[a])
    except:
        print('not enough data')
    print('distance traveled in meters: ' , distance)
    return distance

    #print("current time : ", self.get_clock().now().nanoseconds)
    #print('last speed timestamp: ',self.stamp_vel_s[-1][0])
    #dt = self.get_clock().now().nanoseconds - Time.from_msg(self.speed.header.stamp).nanoseconds 
    #print("time differenc speed/current ", float(dt/1000000000)) 
    #print('closest start timestamp: ', self.stamp_vel_s[start_element], ' element: ', start_element)
    #print('closest end timestamp: ', self.stamp_vel_s[end_element], ' element: ', end_element)

    #print('Detection timestamp: ',det_heaader)
    #print('closes timestamp: ', start_time[0])
    # dt = Time.from_msg(self.Detections.header.stamp).nanoseconds - start_time[0]
    # print('difference between closest and orginal: ', dt/1000000000)
    #print('start pose: ', start_pos)
    #print('start possss: ', start_time[2])         
    #print('last element: ', end_pos)
    #print('the location of this element: ', end_pos)

def compensate_speed(self): # currently not in use now compensate traveled distance when sending the detection to topic detection
    try:
        range_weed = range(self.i_weed)
        for i in range_weed:
            t_new = TransformStamped()
            t_old = get_transform(self, 'weed_removal_tool', 'weed_' + str(i))  # get transform from the weeds to tool

            t_new.header.frame_id =  'weed_removal_tool'
            t_new.child_frame_id =  "weed_tool_" + str(i) 

            seconds = int(self.stamp_vel_s[-1][0]/1000000000)  
            nanosec = self.stamp_vel_s[-1][0] - (seconds*1000000000)
            print(self.stamp_vel_s[-1][0])
            print(seconds, " ", nanosec)

            t_new.header.stamp._sec = seconds
            t_new.header.stamp.nanosec = nanosec
            t_new.transform.translation.x = - t_old.transform.translation.x + traveled_distance(self)   #compensate for speed
            t_new.transform.translation.y = - t_old.transform.translation.y
            t_new.transform.translation.z = - t_old.transform.translation.z
            t_new.transform.rotation.x = 0.0
            t_new.transform.rotation.y = 0.0
            t_new.transform.rotation.z = 0.0
            t_new.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t_new)
            self.i_weed_tool = self.i_weed
        range_crop = range(self.i_crop)

        for y in range_crop:
            t_new = TransformStamped()
            t_old = get_transform(self, 'weed_removal_tool', 'crop_' + str(y))  # get transform from crop
            t_new.header.frame_id =  "weed_removal_tool"
            t_new.child_frame_id =  "crop_tool_" + str(y)        
            seconds = int(self.stamp_vel_s[-1][0]/1000000000)  
            nanosec = self.stamp_vel_s[-1][0] - (seconds*1000000000)
            print(self.stamp_vel_s[-1][0])
            print(seconds, " ", nanosec)
            t_new.header.stamp._sec = seconds
            t_new.header.stamp.nanosec = nanosec
            t_new.transform.translation.x = - t_old.transform.translation.x + traveled_distance(self)   # compensate speed
            t_new.transform.translation.y = - t_old.transform.translation.y
            t_new.transform.translation.z = - t_old.transform.translation.z
            t_new.transform.rotation.x = 0.0
            t_new.transform.rotation.y = 0.0
            t_new.transform.rotation.z = 0.0
            t_new.transform.rotation.w = 1.0
            self.i_crop_tool = self.i_crop

            self.tf_broadcaster.sendTransform(t_new)
    except TransformException as ex:
        self.get_logger().info(
                    f'Could not transform: {ex}')
        return
    except (LookupException, ConnectivityException, ExtrapolationException):
        self.get_logger().info('transform not ready')
        raise
        return 

def transform_frame(self):
        self.i = 0
        self.i_weed = 0
        self.i_crop = 0
        
        tt = get_transform(self, 'weed_removal_tool', 'camera')
        t_new = TransformStamped()
        
        size = range(len(self.detected_list)) 
        
        for i in size:
            object = self.detected_list[i]
            t_new = TransformStamped()
            #t_new.header.stamp = self.get_clock().now().to_msg()  
            t_new.header.stamp = self.Detections.header.stamp 
            t_new.header.frame_id = 'camera'
            if object[0] == 'weed':
                t_new.child_frame_id = object[0] + "_" + str(self.i_weed)
                self.i_weed = self.i_weed + 1
            else:
               t_new.child_frame_id = object[0] + "_" + str(self.i_crop) 
               self.i_crop = self.i_crop + 1
            
            t_new.transform.translation.x = object[1] 
            t_new.transform.translation.y = object[2] 
            t_new.transform.translation.z = object[3] 
            t_new.transform.rotation.x = tt.transform.rotation.x
            t_new.transform.rotation.y = tt.transform.rotation.y
            t_new.transform.rotation.z = tt.transform.rotation.z
            t_new.transform.rotation.w = tt.transform.rotation.w
            self.i = self.i +1
            self.tf_broadcaster.sendTransform(t_new)
            
            # print("orginal detection: ",  t_new.child_frame_id)
            # print("Position:")
            # print("     x: ", object[1] )
            # print("     y: ", object[2] )
            # print("     z: ", object[3] )




            #t_new.transform.translation.x = float(distance)
            #t_new.transform.translation.y = 0.0 
            #t_new.transform.translation.z = 0.0
            #t_new.transform.rotation.x = 0.0
            #t_new.transform.rotation.y = 0.0
            #t_new.transform.rotation.z = 0.0
            #t_new.transform.rotation.w = 1.0
            
            #self.tf_broadcaster.sendTransform(t_new)
           # tnn = get_transform(self, "camera", t_new.child_frame_id)
           # print("new pose: ",  t_new.child_frame_id)
            #print("Position:")
          #  print("     x: ", tnn.transform.translation.x)
           # print("     y: ", tnn.transform.translation.y)
            #print("     z: ", tnn.transform.translation.z)
        
def pub_detections_tool_frame(self): # detection form camera to tool frame
    
    msg = Detections()
    msg.header.stamp = self.Detections.header.stamp

    if self.i_weed != 0:
        range_weed = range(self.i_weed)
        for a in range_weed: 
            t_old = get_transform(self, 'weed_removal_tool', "weed_" + str(a))

            # print('detect header stamp: ', self.Detections.header.stamp)
            # print('tf header stamp: ', Time.from_msg(t_old.header.stamp).nanoseconds)
            # print('current time stamp: ', self.get_clock().now().nanoseconds )
        #print(t_old)
            x = t_old.transform.translation.x * (-1) + traveled_distance(self, Time.from_msg(t_old.header.stamp).nanoseconds, self.get_clock().now().nanoseconds ) # compensating for the speed of agrobot
            y = t_old.transform.translation.y * (-1)
            z = t_old.transform.translation.z * (-1)

            msg.detected.append(detection_to_msg('weed', x ,y ,z))

    
    if self.i_crop != 0:
        range_crop = range(self.i_crop)
        for b in range_crop:
            t_old = get_transform(self, 'weed_removal_tool', "crop_" + str(b))
            x = t_old.transform.translation.x * (-1) + traveled_distance(self, Time.from_msg(t_old.header.stamp).nanoseconds, self.get_clock().now().nanoseconds ) # compensating for the speed of agrobot
            y = t_old.transform.translation.y * (-1)
            y = t_old.transform.translation.y * (-1)
            z = t_old.transform.translation.z * (-1)

            msg.detected.append(detection_to_msg('crop', x ,y ,z))

        self.publisher.publish(msg)
    #print_detections(msg)
    return

def broadcast_timer_callback(self):
        
        try:
            pub_detections_tool_frame(self)     # here the placed points get read using lookup_transform and then coordinates get published to topic
        except:
            print("Failed to find a transform to ")     
        transform_frame(self) # here points get placed into the world

        # self.tf_listener.lookupTransform('weed_removal_tool','weed_0', 0)
        #LookupTransform('weed_removal_tool','weed_0', 0)
        #pub_detections_tool_frame(self)
        # if self.startup == 1:
        #     pub_detections_tool_frame(self)
        #     transform_frame(self) 
        #     print('publishing coordinates 2e',self.startup) 
        # # elif self.startup == 1:          
        # #     compensate_speed(self)  
        # #     self.startup= self.startup+1
        # #     print('compensate speed 2e',self.startup)
        # else:   
        #     transform_frame(self) 
        #     #pub_detections_tool_frame(self)
        #     self.startup = self.startup+1
        #     print("transform frame 1e: ", self.startup)
        # transform_frame(self) 
        # t = self.tf_buffer.can_transform('weed_removal_tool',
        #                                             'weed_0',
        #                                             self.get_clock().now())

        # tt = self.tf_buffer.lookup_transform('weed_removal_tool',
        #                                             'weed_0',
        #                                             self.Detections.header.stamp,
        #                                             rclpy.duration.Duration(seconds=10.0))
        # print(t)
        # print(tt)
        # try:
                
        #     t = self.tf_buffer.canTransform('weed_removal_tool',
        #                                             'weed_0',
        #                                             0)
        #     print(t)
        # except:
        #     print("Failed to find a transform to ") 
            

class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('transforming_coordinate_frames')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.detected_list = []
        self.i = 0
        self.i_weed = 0
        self.i_crop = 0
        self.startup = False        
        
        self.stamp_vel_s = []
        self.Detections = Detections()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_detections = self.create_subscription( #create subscriber
            Detections,
            'detections',
            self.listener_callback_detections,
            10)
        self.sub_detections # prevent unused variable warning

        
        self.sub_speed = self.create_subscription(  #create subscriber
            TwistStamped,
            'speed',
            self.listener_callback_speed,
            10)
        self.sub_speed # prevent unused variable warning 
        self.publisher = self.create_publisher(Detections, 'detections_tool_frame', 10)
        #self.timer = self.create_timer(0.1,  pub_detections_tool_frame(self))
       # self.timer = self.create_timer(0.1, self.timer_callback)
        
    def listener_callback_speed(self, Speed):           # save speed from speed topic
        self.speed = Speed
        dt = self.get_clock().now().nanoseconds - Time.from_msg(self.speed.header.stamp).nanoseconds  
        distance(self)

    def listener_callback_detections(self, Detections): # obtain detections from detections topic and execute publish_filter_data function
        self.Detections = Detections
        self.detected_list = detections_to_list(Detections)

        broadcast_timer_callback(self)
        
        #transform_frame(self)
        #self.startup = True 
        #print(self.startup)

        
    # def timer_callback(self):
    #     try:
    #         if self.startup == True:
    #             pub_detections_tool_frame(self)
    #             self.startup == False
    #             print(self.startup)

    #         else:
    #             self.startup == False
    #     except:
    #         print('transform not found')
        
        
        
    
    # def get_transform(self):
        
    #     try:
    #         pub_detections_tool_frame(self)     # here the placed points get read using lookup_transform and then coordinates get published to topic
    #     except:
    #         print("Failed to find a transform to ")     
    #     #transform_frame(self) # here points get placed into the world

def main():
    rclpy.init()
    
    node = DynamicFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
