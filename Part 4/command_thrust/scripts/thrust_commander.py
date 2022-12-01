#!/usr/bin/env python
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
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
from __future__ import print_function
import rospy
import numpy as np
from std_msgs.msg import Bool
from gazebo_msgs.msg import *
from geometry_msgs.msg import Wrench, Vector3, Point

def readinputs():
	rospy._init_node_('listener',anonymous=True)
	#subscribe from /rexrov/thruster_manager/input
	rospy.Subscriber("/rexrov/thruster_manager/input", Wrench, publish_thrusters)
	rospy.spin()
	
	
def wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration):
  #  Call thrust commands as the service /gazebo/apply_body_wrench

    rospy.wait_for_service('/gazebo/apply_body_wrench')
    try:
        # proxy object to call /gazebo/apply_body_wrench' service
        apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        apply_body_wrench(body_name, reference_frame, reference_point, wrench, start_time, duration)
    except rospy.ServiceException as e:
        print("failed to call service %s" % e)
        
        
def publish_thrusters(data):
    #Callback function for listener, which publishes thruster commands using apply_body_wrench_client given respective data
    
    rospy.loginfo(rospy.get_caller_id() + "Forces: %s", data.force)
    rospy.loginfo(rospy.get_caller_id() + "Torques: %s", data.torque)
    wrenchobj = Wrench(
        force=Vector3(
            x=data.force.x,
            y=data.force.y,
            z=data.force.z),
        torque=Vector3(
            x=data.torque.x,
            y=data.torque.y,
            z=data.torque.z))
    # call required service using apply_body_wrench_client
    wrench_client('robot_rov::base_link', "", Point(x=0, y=0, z=0), wrenchobj, rospy.Time(secs=0, nsecs=0), rospy.Duration(secs=1, nsecs=0))


if __name__ == '__main__':
    # start to read the inputs
    readinputs()
    
    
    
    
    
    
    
