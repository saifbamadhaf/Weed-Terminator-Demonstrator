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
import yaml
import os

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ament_index_python import get_package_share_directory

# import generic tool interface messages   
from std_srvs.srv import Trigger
from detection_interfaces.srv import TriggerDetection  
from detection_interfaces.msg import Detected, Detections

def msg_to_detection(detection_msg):    # Detection messaged converted back to variables
    type = detection_msg.type
    x = detection_msg.position.x
    y = detection_msg.position.y 
    z = detection_msg.position.z       
    return type, x, y, z 

def detection_to_msg(type, x, y, z):    # Converts variables to a msg Detected() this message can then be placed in in detections message
    detection_msg = Detected()
    detection_msg.type = type
    detection_msg.position.x = x
    detection_msg.position.y = y
    detection_msg.position.z = z       
    return detection_msg 

def print_detection(detection_msg):   # Prints the detections in terminal
    type, x, y, z = msg_to_detection(detection_msg)
    print("Detected", type, "at x", x, "y", y, "z",z)

class MeasurementService(Node):

    def __init__(self):
        super().__init__('measurement_service')
        
        self.clear_list_on_measurement = True

        # Declare and acquire `reference_frame` and `target_frame` parameters
        self.reference_frame = self.declare_parameter('reference_frame', 'root_link').get_parameter_value().string_value
        self.target_frame = self.declare_parameter('target_frame', 'laser').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        package_path = get_package_share_directory('detection_protocol')
        absolute_file_path = os.path.join(package_path, 'config/measured_positions.yaml')

        self.declare_parameter('filename', absolute_file_path)

        self.callback_group = ReentrantCallbackGroup()
        self.subscriber = self.create_service(TriggerDetection, '/joystick/measure_plant', self.measurement_callback, callback_group=self.callback_group)
        self.subscriber
        self.send_list_service = self.create_service(Trigger, '/joystick/send_list', self.send_list_callback, callback_group=self.callback_group)
        self.send_list_service
        
        self.publisher_ = self.create_publisher(Detections, 'filtered_detections', 10) # create publisher for topic /detections which is a message type detections
        
        self.filename = self.get_parameter('filename').get_parameter_value().string_value

        file = open(self.filename, 'r')
        data = yaml.safe_load(file)
        self.detection_list = data['detections']

        self.get_logger().info('Started measurement service listening to measurement requests.')
        self.get_logger().info("Loaded %d detections from %s" % (len(self.detection_list), self.filename))
        
    def measurement_callback(self, request, response):

        if (self.clear_list_on_measurement):
            self.detection_list = []
            self.clear_list_on_measurement = False
            self.get_logger().info('Start with empty list of weeds and crops')

        # Look up for the transformation between target frame and reference frame
        try:
            t = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.target_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.reference_frame} to {self.target_frame}: {ex}')
            return response
        
        detection = Detected()
        detection.type = request.type
        detection.position.x = t.transform.translation.x
        detection.position.y = t.transform.translation.y
        detection.position.z = t.transform.translation.z
        
        self.detection_list.append(msg_to_detection(detection))
        print_detection(detection)

        data = {'detections': self.detection_list}

        file = open(self.filename, 'w')
        yaml.safe_dump(data, file)

        print ("Saved %d detections to %s" % (len(self.detection_list), self.filename))

        response.position = detection.position
        return response

    def send_list_callback(self, request, response):
        self.clear_list_on_measurement = True

        msg = Detections()

        for i in range(len(self.detection_list)):
            msg.detected.append(detection_to_msg(self.detection_list[i][0], 
                                            self.detection_list[i][1], 
                                            self.detection_list[i][2], 
                                            self.detection_list[i][3]))

        self.publisher_.publish(msg)

        response.success = True
        response.message = "Published list of weeds and crops"

        return response

def main(args=None):
    rclpy.init(args=args)

    measure_positions = MeasurementService()
    executor = MultiThreadedExecutor()
    executor.add_node(measure_positions)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print('Node stopped. Saving data...')

    data = {'detections': measure_positions.detection_list}

    file = open(measure_positions.filename, 'w')
    yaml.safe_dump(data, file)

    print ("Saved %d detections to %s" % (len(measure_positions.detection_list), measure_positions.filename))
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    measure_positions.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
