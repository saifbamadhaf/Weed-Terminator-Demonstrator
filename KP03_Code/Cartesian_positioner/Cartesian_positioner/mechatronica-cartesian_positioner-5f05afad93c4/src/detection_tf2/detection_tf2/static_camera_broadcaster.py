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

from geometry_msgs.msg import TransformStamped
import math
import rclpy

import numpy as np
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class FixedFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('static_cam_broadcaster')
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.make_transforms()

    def make_transforms(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()    # give coordinate a frame name and timestamp its related to weed removal tool
        t.header.frame_id = 'weed_removal_tool'
        t.child_frame_id= 'camera'

        t.transform.translation.x = -1.6        # the camera is placed 1.6 meters in front of the tool and in the middel.
        t.transform.translation.y = 0.35
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(
            float(0), float(-(np.pi*(3/4))), float(0))      #rotate 45 degrees around the y-axis
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)                # places coordinate frame in the tf2 world


def main():
    logger = rclpy.logging.get_logger('logger')
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
