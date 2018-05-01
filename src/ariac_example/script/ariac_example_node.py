#!/usr/bin/env python
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




from __future__ import print_function
from ariac_example import ariac_example
from tf.transformations import quaternion_from_euler
import math
import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from osrf_gear.msg import Order
from osrf_gear.msg import VacuumGripperState
from osrf_gear.srv import ConveyorBeltControl
from osrf_gear.srv import DroneControl
from osrf_gear.srv import VacuumGripperControl
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
import xml.etree.ElementTree as ET
import sys, tf
#import tf2
#import tf2_geometry_msgs
import Subscriber





def main():

    sub = Subscriber.Subscriber()
    rospy.init_node("_ariac_competion_node_")
    rospy.sleep(2)
    #armcontroll = ariac_example.ArmControll()
    Start()
    rospy.loginfo("=============Setup complete.")
    #armcontroll.sendOverBin(3)

    invpose = geometry_msgs.msg.Pose()    
    invpose.position.x=0.521671259388
    invpose.position.y=-0.184750284639
    invpose.position.z= 0.164743542447
    invpose.orientation.x = 0.62185046715
    invpose.orientation.y = 0.00667755907235
    invpose.orientation.z =0.78309805686
    invpose.orientation.w =-0.00385227873479
    #pose = armcontroll.transformPose(invpose, [-0.02,0,0], [0,0,0,0], 'logical_camera_1_frame')
    #armcontroll.poseTarget(pose)
    #arm.planPose()
    #arm.executePlan()


    
    
    
    #armcontroll.grabPart()

   # while not rospy.is_shutdown():
    #    if():
            




    
    
    
    rospy.spin()



def Start():
    rospy.loginfo("Waiting for competition to be ready...")
    rospy.wait_for_service('/ariac/start_competition')
    rospy.loginfo("Competition is now ready.")
    rospy.loginfo("Requesting competition start...")

    try:
        start = rospy.ServiceProxy('/ariac/start_competition', Trigger)
        response = start()
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to start the competition: %s" % exc)
        return False
    if not response.success:
        rospy.logerr("Failed to start the competition: %s" % response)
    else:
        rospy.loginfo("Competition started!")
    return response.success






if __name__ == '__main__':
    main()



