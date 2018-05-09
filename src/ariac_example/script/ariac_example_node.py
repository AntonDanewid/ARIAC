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
from ariac_example import ariac_planner




def main():


    #Fix name to valid
    rospy.init_node("_ariac_competion_node_")

    #armcontroll = ariac_example.ArmControll()
    planner = ariac_planner.Planner()
    rospy.sleep(5)

    while not rospy.is_shutdown():
        if len(planner.recieved_orders) > 0:
            currentOrder = planner.recieved_orders[0]

            #Check if there are enough parts to fullfill order 
            possibleToBuild = enoughParts(currentOrder, planner)
            if not possibleToBuild:
                planner.recieved_orders.pop(0)   
            while currentOrder.products > 0 and possibleToBuild:
                currentProduct = currentOrder.products[0]
                #Find location of current product
                
                #Send arm over the bin of the current product
                #Send arm to the product location
                #Attach product to vaccum
                #Check if product is attached
                #Move arm avway from bin
                #Move arm over shipping box
                #Place product in correct position
                #Check if part is faulty
                #If faulty, remove from box, somewhere where it dosnt collide
                    #If faulty product affects order completion, put back all parts
                        #Remove order and restart with a new order
                #Set boolean of completed to true    
                #End While
            #If completed, start conveyer belt, call drone and ship.
            #Repeat all over again
            #



            

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
    pose = sub.getLocationOfPart("gear_part")
    #print(pose.position.x)
    #sub.translatePose(pose, 'logical_camera_1')

    print(sub.getAmountOfParts("gear_part"))

    #pose = armcontroll.transformPose(invpose, [-0.02,0,0], [0,0,0,0], 'logical_camera_1_frame')
    #armcontroll.poseTarget(pose)
    #arm.planPose()
    #arm.executePlan()


    
   
            
def enoughParts(currentOrder, planner)
    prodDict = {}
    for product in currentOrder.products:
        if(product.name in prodDict):
            prodDict[product.name] = prodDict[product.name] +1
        else: 
            prodDict[product.name] = 0
        for productName in prodDict:
            if planner.getAmountOfParts(productName) < prodDict[productName]:
            #We cant fullfill this order because we are missing parts
                return False
    return True
    
    





if __name__ == '__main__':
    main()



