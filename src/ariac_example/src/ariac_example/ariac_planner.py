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

import time

import rospy
import Orders
import Product
from osrf_gear.msg import Order
from osrf_gear.msg import Product
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
from osrf_gear.srv import GetMaterialLocations
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg

from osrf_gear.msg import LogicalCameraImage
import sys, tf

import tf2_ros
import tf2_geometry_msgs

def start_competition(planner):
    rospy.loginfo("Waiting for competition to be ready...")
    rospy.wait_for_service('/ariac/start_competition')
    rospy.loginfo("Competition is now ready.")
    rospy.loginfo("Requesting competition start...")

    try:
        start = rospy.ServiceProxy('/ariac/start_competition', Trigger)
        response = start()
        planner.conveyor_isactive = True
        control_conveyor(100)
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to start the competition: %s" % exc)
    if not response.success:
        rospy.logerr("Failed to start the competition: %s" % response)
    else:
        rospy.loginfo("Competition started!")
    #order_sub1 = rospy.Subscriber("/ariac/orders", Order, planner.order_handle)
    rospy.loginfo("Subscribed to orders!")
    order_sub2 = rospy.Subscriber("/ariac/break_beam_1_change", Proximity, planner.control_drone)
    rospy.loginfo("Subscribed to break_beam_1!")
    order_sub3 = rospy.Subscriber("/ariac/arm_planner_out", String, planner.product_shute)
    rospy.loginfo("Publisher to arm_planner_in initiated")
    #order_sub4 = rospy.Subscriber("/ariac/arm_planner_in", Product, planner.product_shute)
    #rospy.loginfo("Publisher to arm_planner_in initiated")

def control_conveyor(power):
    rospy.loginfo("Waiting for conveyor control to be ready...")
    name = '/ariac/conveyor/control'
    rospy.wait_for_service(name)
    rospy.loginfo("Conveyor control is now ready.")
    rospy.loginfo("Requesting conveyor control...")

    try:
        conveyor_control = rospy.ServiceProxy(name, ConveyorBeltControl)
        response = conveyor_control(power)
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to control the conveyor: %s" % exc)
        return False
    if not response.success:
        rospy.logerr("Failed to control the conveyor: %s" % response)
    else:
        rospy.loginfo("Conveyor controlled successfully")
    return response.success



class Planner:
    def __init__(self):
        self.counter = 0
        self.recieved_orders = []
        self.current_ordered_parts = []
        self.current_completed_parts = []
        self.current_part = 0
        self.completed_orders = []
        self.conveyor_isactive = False
        rospy.loginfo("Subscribed to arm_planner_out!")
        self.arm_commander = rospy.Publisher("/ariac/arm_planner_in", Product, queue_size=10)
        self.order_sub = rospy.Subscriber("/ariac/orders", Order, self.order_handle)
        self.currentOrders = []
        self.current_comp_state = None
        self.received_orders = []
        self.current_joint_state = None
        self.current_gripper_state = None
        self.last_joint_state_print = time.time()
        self.last_gripper_state_print = time.time()
        self.materialLocationsService1 = rospy.ServiceProxy('/ariac/material_locations',  GetMaterialLocations)
        self.logicalCameraData1 = None
        self.pastLogicalCameraTime1 = rospy.get_time()
        self.logicalCameraData2 = None
        self.pastLogicalCameraTime2 = rospy.get_time()

        self.logicalCameraData3 = None
        self.pastLogicalCameraTime3 = rospy.get_time()


        self.logicalCameraSubscriber1 = rospy.Subscriber("/ariac/logical_camera_1", LogicalCameraImage, self.logicalCameraEvent1)
        self.logicalCameraSubscriber2 = rospy.Subscriber("/ariac/logical_camera_4", LogicalCameraImage, self.logicalCameraEvent2)
        self.logicalCameraSubscriber3 = rospy.Subscriber("/ariac/logical_camera_3", LogicalCameraImage, self.logicalCameraEvent3)

        self.tf_listener = tf.TransformListener()




    #triggered when brake_beam_1 is broken. Will stop the conveyor, call for a
    #drone to pick up a package if one is pressent and ask the arm to start
    #fullfilling the next order if there is one.
    def control_drone(self, shipment_type):
        #print("switching it up:",self.counter)
        self.counter = self.counter+1
        if self.counter%2 is 1:
            control_conveyor(0)
            self.conveyor_isactive = False
            time.sleep(4)
            self.start_arm()
            shipment_type = "package recieved"
            rospy.loginfo("Waiting for drone control to be ready...")
            name = '/ariac/drone'
            rospy.wait_for_service(name)
            rospy.loginfo("Drone control is now ready.")
            rospy.loginfo("Requesting drone control...")

            try:
                drone_control = rospy.ServiceProxy(name, DroneControl)
                response = drone_control(shipment_type)
            except rospy.ServiceException as exc:
                rospy.logerr("Failed to control the drone: %s" % exc)
                return False
                if not response.success:
                    rospy.logerr("Failed to control the drone: %s" % response)
                else:
                    rospy.loginfo("Drone controlled successfully")
                    return response.success

    #requests the arm to start working on a new order and pick up a product to
    #put in the box. Will do nothing if the conveyor is running or if there are
    #no orders available. If current_ordered_parts is empty this method will
    #fill it with the next order.
    def start_arm(self):
        rospy.loginfo("start_arm triggered... recieved orders are: ")
        print(len(self.recieved_orders))
        print(len(self.recieved_orders) > 0 and not self.conveyor_isactive)
        if(len(self.recieved_orders) > 0 and not self.conveyor_isactive):
            if(len(self.current_ordered_parts) == 0):
                self.current_ordered_parts = self.recieved_orders[0].products
            self.current_part = self.current_ordered_parts[0]
            self.current_ordered_parts = self.current_ordered_parts[1:]
            #self.arm_commander.publish(self.current_part)
            #here we make the arm move mechanicly lel
            rospy.loginfo("Product request published to arm..")

    #is run when a new order is recieved. appends the order to the local list,
    #and begin handling the order through start_arm() if no other orders are
    #currently beeing handled.
    def order_handle(self, msg):

        rospy.loginfo("Order recieved. HERE")
        self.recieved_orders.append(Orders.Orders(msg))
        # rospy.loginfo(msg.order_id)
        # if(len(self.recieved_orders) == 1 and not self.conveyor_isactive):
        #     self.start_arm()

    #is run when the arm acks back. When this happens another product is
    #requested if there are more in the order. Otherwise the conveyor is started
    def product_shute(self, msg):
        rospy.loginfo("Arm reported finished with transporting.")
        self.current_completed_parts.append(self.current_part)
        if(len(self.current_ordered_parts) > 0):

            self.current_part = self.current_ordered_parts[0]
            self.arm_commander.publish(self.current_ordered_parts[0])
            self.current_ordered_parts = self.current_ordered_parts[1:]
        else:
            rospy.loginfo("Order completed. Starting conveyor.")
            self.current_completed_parts = []
            control_conveyor(100)
            self.conveyor_isactive = True
            self.completed_orders.append(self.recieved_orders[0])
            self.recieved_orders = self.recieved_orders[1:]



    def joint_state_callback(self, msg):
        if time.time() - self.last_joint_state_print >= 10:
            rospy.loginfo("Current Joint States (throttled to 0.1 Hz):\n" + str(msg))
            self.last_joint_state_print = time.time()
        self.current_joint_state = msg


    def start_competition(self, planner):
        rospy.loginfo("Waiting for competition to be ready...")
        rospy.wait_for_service('/ariac/start_competition')
        rospy.loginfo("Competition is now ready.")
        rospy.loginfo("Requesting competition start...")
        try:
            start = rospy.ServiceProxy('/ariac/start_competition', Trigger)
            response = start()
            planner.conveyor_isactive = True
            control_conveyor(100)
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to start the competition: %s" % exc)
        if not response.success:
            rospy.logerr("Failed to start the competition: %s" % response)
        else:
            rospy.loginfo("Competition started!")
        order_sub1 = rospy.Subscriber("/ariac/orders", Order, self.order_handle)
        rospy.loginfo("Subscribed to orders!")
        order_sub2 = rospy.Subscriber("/ariac/break_beam_1_change", Proximity, self.control_drone)
        rospy.loginfo("Subscribed to break_beam_1!")
        order_sub3 = rospy.Subscriber("/ariac/arm_planner_out", String, self.product_shute)
        rospy.loginfo("Publisher to arm_planner_in initiated")
        
      #Returns the pose of a requested part in local coordinate system. Needs tranforms
    def getLocationOfPart(self, part):
        for model in self.logicalCameraData1.models:
            if model.type == part:
                pose = PoseStamped()
                pose.header.frame_id = 'logical_camera_1'              
                pose.pose.position.x = model.pose.position.x
                pose.pose.position.y = model.pose.position.y
                pose.pose.position.z = model.pose.position.z
                pose.pose.orientation.x = model.pose.orientation.x
                pose.pose.orientation.y = model.pose.orientation.y
                pose.pose.orientation.z = model.pose.orientation.z
                pose.pose.orientation.w = model.pose.orientation.w
                return pose
        for model in self.logicalCameraData2.models:
            if model.type == part:
                pose = PoseStamped()
                pose.header.frame_id = 'logical_camera_2'
                pose.pose.position.x = model.pose.position.x
                pose.pose.position.y = model.pose.position.y
                pose.pose.position.z = model.pose.position.z
                pose.pose.orientation.x = model.pose.orientation.x
                pose.pose.orientation.y = model.pose.orientation.y
                pose.pose.orientation.z = model.pose.orientation.z
                pose.pose.orientation.w = model.pose.orientation.w
                return pose
        for model in self.logicalCameraData3.models:
            if model.type == part:
                pose = PoseStamped()
                pose.header.frame_id = 'logical_camera_3'
                pose.pose.position.x = model.pose.position.x
                pose.pose.position.y = model.pose.position.y
                pose.pose.position.z = model.pose.position.z
                pose.pose.orientation.x = model.pose.orientation.x
                pose.pose.orientation.y = model.pose.orientation.y
                pose.pose.orientation.z = model.pose.orientation.z
                pose.pose.orientation.w = model.pose.orientation.w
                return pose


    #Adds a logical camera message
    def logicalCameraEvent1(self, msg):
        now = rospy.get_time()
        if self.pastLogicalCameraTime1 + 1.0 < now and len(msg.models) > 0:
            # Log camera
            #rospy.loginfo("Logic Camera: " + str(len(msg.models)) + " Objects")
            # Set new past time
            self.pastLogicalCameraTim1e = rospy.get_time()
        self.logicalCameraData1 = msg
    
    def logicalCameraEvent2(self, msg):
        now = rospy.get_time()
        if self.pastLogicalCameraTime2 + 1.0 < now and len(msg.models) > 0:
            # Log camera
            #rospy.loginfo("Logic Camera: " + str(len(msg.models)) + " Objects")
            # Set new past time
            self.pastLogicalCameraTime2 = rospy.get_time()
        self.logicalCameraData2 = msg


    def logicalCameraEvent3(self, msg):
        now = rospy.get_time()
        if self.pastLogicalCameraTime3 + 1.0 < now and len(msg.models) > 0:
            # Log camera
            #rospy.loginfo("Logic Camera: " + str(len(msg.models)) + " Objects")
            # Set new past time
            self.pastLogicalCameraTime3 = rospy.get_time()
        self.logicalCameraData3 = msg

    #Translates a local pose to world pose from the frame provided
    #Fix problem with nonexisting frame
    def translatePose(self, pose, frame):
        # targetPose = PoseStamped()
        # targetPose.header.frame_id = pose.pose.header.frame_id
        # targetPose.pose.position.x = pose.pose.position.x
        # targetPose.pose.position.y = pose.pose.position.y
        # targetPose.pose.position.z = pose.pose.position.z
        # targetPose.pose.orientation.x = 0.0
        # targetPose.pose.orientation.y = 0.0
        # targetPose.pose.orientation.z = 0.0
        # targetPose.pose.orientation.w = 0.0
        transformedPose = self.tf_listener.transformPose('world', pose)
        print(transformedPose)

    #Add for all the different sensors, right now it can only do for the single logical camera
    def getAmountOfParts(self, partName):
        amount = 0
        for model in self.logicalCameraData1.models:
            if model.type == partName:
                print("FOUND PART")
                amount +=1
        for model in self.logicalCameraData2.models:
            if model.type == partName:
                print("FOUND PART")
                amount +=1
        for model in self.logicalCameraData3.models:
            if model.type == partName:
                print("FOUND PART")
                amount +=1
        return amount


