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
from osrf_gear.srv import GetMaterialLocations
from osrf_gear.msg import LogicalCameraImage
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
import xml.etree.ElementTree as ET
import sys, tf
import Orders

class Subscriber: 

    def __init__(self):
        self.order_sub = rospy.Subscriber("/ariac/orders", Order, self.orderReceived)
        self.currentOrders = []
        self.current_comp_state = None
        self.received_orders = []
        self.current_joint_state = None
        self.current_gripper_state = None
        self.last_joint_state_print = time.time()
        self.last_gripper_state_print = time.time()
        self.materialLocationsService1 = rospy.ServiceProxy('/ariac/material_locations',  GetMaterialLocations)
        self.logicalCameraData = None
        self.pastLogicalCameraTime = rospy.get_time()
        self.logicalCameraSubscriber = rospy.Subscriber("/ariac/logical_camera_1", LogicalCameraImage, self.logicalCameraEvent)





    def orderReceived(self, order):
        o = Orders.Orders(order)
        self.currentOrders.append(o)

    def getLocationOfPart(self, part): 
        print(self.logicalCameraData.models)
        for model in self.logicalCameraData.models:
            if model.type == part:
                pose = geometry_msgs.msg.Pose()
                pose.position.x = model.pose.position.x
                pose.position.y = model.pose.position.y
                pose.position.z = model.pose.position.z
                pose.orientation.x = model.pose.orientation.x
                pose.orientation.y = model.pose.orientation.y
                pose.orientation.z = model.pose.orientation.z
                pose.orientation.w = model.pose.orientation.w
                return pose

    
    def logicalCameraEvent(self, msg):
        now = rospy.get_time()
        if self.pastLogicalCameraTime + 1.0 < now and len(msg.models) > 0:
            # Log camera
            #rospy.loginfo("Logic Camera: " + str(len(msg.models)) + " Objects")
            # Set new past time
            self.pastLogicalCameraTime = rospy.get_time()
        
        self.logicalCameraData = msg
    
    