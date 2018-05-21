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
from osrf_gear.msg import LogicalCameraImage
from osrf_gear.msg import Proximity
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
import ariac_planner
import ariac_arm
#from ariac_example import start_competition



def main():

    #Fix name to valid#
    rospy.init_node("ariac_competion_node")
    armcontroll = ariac_arm.ArmControll()
    planner = ariac_planner.Planner()
    rospy.sleep(5)
    #armcontroll.send_start()
    ariac_planner.start_competition(planner)
    rospy.loginfo("=============Setup complete.")
    break_beam_sub = rospy.Subscriber("/ariac/break_beam_1_change", Proximity, planner.callWhenBeamBreaks)
    print("=============Subscribed to break_beam_1.")
    planner.control_conveyor(100)
    time.sleep(1)
    print("=============waiting for conveyor to stop.")
    while(planner.conveyor_isactive):
        time.sleep(1)
    print("========starting for real-real.")
    placedProducts = []

    while not rospy.is_shutdown():
        if len(planner.recieved_orders) > 0:
            currentOrder = planner.recieved_orders[0]
            print("===============WE HAVE AN ORDER")
            for product in currentOrder.products:
                print("=========", product.name)
            #Check if there are enough parts to fullfill order
            possibleToBuild = enoughParts(currentOrder, planner)
            if not possibleToBuild:
                print("===============NOT ENOUGH PARTS, ORDER DUMPED")
                planner.recieved_orders.pop(0)
                for part in placedProducts:
                    #remove from bin
                    pass
            else:
                print("===============WE CAN BUILD THIS ORDER")
            placedProducts = []
            while len(currentOrder.products) > 0 and possibleToBuild:
                while(planner.conveyor_isactive):
                    time.sleep(1)
                currentProduct = currentOrder.products[0]
                #Get a location for a product in cameras local coordinate system
                #print(currentProduct.name)
                productPose = planner.getLocationOfPart(currentProduct.name)
                                #Transform the coordinates to world coordinatses
                worldPose = planner.translatePose(productPose)
                #Locate which bin the part is in
                print('================== FRAME ID, ' , productPose.header.frame_id)
                if "1" in productPose.header.frame_id:
                    bin = 3
                elif "3" in productPose.header.frame_id:
                    bin = 4
                elif "4" in productPose.header.frame_id:
                    bin = 2
                elif "5" in productPose.header.frame_id:
                    bin = 1
                elif "6" in productPose.header.frame_id:
                    bin = 5     
                print("=================== THE BIN WE ARE MOVING TO IS BIN ", bin)
                #Send arm over the bin of the current product. THIS DOES NOT WORK CORRECTLY AT THE MOMENT
                #Send arm to the product location
                armcontroll.sendOverBin(bin)
                while not armcontroll.gripperStateData.attached:
                    armcontroll.grabPart(worldPose, bin)
                    armcontroll.aboveBin(bin)
                    #rospy.sleep(1)
                #Attach product to vaccum
                armcontroll.sendBackFromBin(bin)

                #Check if product is attached
                #Move arm avway from bin
                #Move arm over shipping box
                #Place product in correct position
                targetPosition = PoseStamped()
                targetPosition.pose = currentProduct.pose
                targetPosition.header.frame_id = 'shipping_box_frame' #Double check
                    #Move arm
                #worldTarget = planner.translatePose(targetPosition)
                #Did we manage to place the part in the shipping box? 
                suc    = armcontroll.sendOverTray()
                if not suc:
                    break
                    

                #planner.transformToTray()
                #If faulty, remove from box, somewhere where it dosnt collide
                #drop part
                #Save where the part is if we need to throw it away
                placedPose = PoseStamped()
                placedPose.header.frame_id = currentProduct.name
                placedPose.pose = armcontroll.getArmPosition()
                
                placedProducts.append(placedPose)
                #Do we have sensor blackout? Do not ship anything if this is the case. Wait

                while planner.sensorBlackOutCheck(): 
                    rospy.sleep(1)
                if planner.faulty:
                    if not enoughParts(currentOrder, planner):
                        pass
                        #If faulty product affects order completion, put back all parts
                        #Remove order and restart with a new order
                    else:
                        #Just remove the faulty part
                        pass
                #Set boolean of completed to true
                currentOrder.products.pop(0)
                #End While
            #If completed, start conveyer belt, call drone and ship.
            if len(currentOrder.products) == 0:
                #planner.product_shute()
                print("============COMPLETED ORDER")
                planner.recieved_orders.pop(0)
                planner.control_conveyor(100)

            #Repeat all over again




def enoughParts(currentOrder, planner):
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
    rospy.spin()