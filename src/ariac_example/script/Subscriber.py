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
import tf2_ros
import tf2_geometry_msgs

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
        self.tf_listener = tf.TransformListener()

    #Adds order objects to a list
    def orderReceived(self, order):
        self.currentOrders.append(Orders.Orders(order))


    #Returns the pose of a requested part in local coordinate system. Needs tranforms
    def getLocationOfPart(self, part):
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


    #Adds a logical camera message
    def logicalCameraEvent(self, msg):
        now = rospy.get_time()
        if self.pastLogicalCameraTime + 1.0 < now and len(msg.models) > 0:
            # Log camera
            #rospy.loginfo("Logic Camera: " + str(len(msg.models)) + " Objects")
            # Set new past time
            self.pastLogicalCameraTime = rospy.get_time()
        self.logicalCameraData = msg

    #Translates a local pose to world pose from the frame provided
    #Fix problem with nonexisting frame
    def translatePose(self, pose, frame):
        targetPose = PoseStamped()
        targetPose.header.frame_id = 'shipping_box_frame'
        targetPose.pose.position.x = pose.position.x
        targetPose.pose.position.y = pose.position.y
        targetPose.pose.position.z = pose.position.z
        targetPose.pose.orientation.x = 0.0
        targetPose.pose.orientation.y = 0.0
        targetPose.pose.orientation.z = 0.0
        targetPose.pose.orientation.w = 0.0
        transformedPose = self.tf_listener.transformPose('world', targetPose)
        print(transformedPose)

    #Add for all the different sensors, right now it can only do for the single logical camera
    def getAmountOfParts(self, partName):
        amount = 0
        for model in self.logicalCameraData.models:
            if model.type == partName:
                print("FOUND PART")
                amount +=1
        return amount
