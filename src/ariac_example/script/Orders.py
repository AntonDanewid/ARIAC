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
import Product

class Orders:
    def __init__(self, order):
        self.orderID = order.order_id
        self.products = []
        for part in order.shipments[0].products:
            name = part.type
            pose = geometry_msgs.msg.Pose()
            pose.position.x = part.pose.position.x
            pose.position.y = part.pose.position.y
            pose.position.z = part.pose.position.z
            pose.orientation.x = part.pose.orientation.x
            pose.orientation.y = part.pose.orientation.y
            pose.orientation.z = part.pose.orientation.z
            pose.orientation.w = part.pose.orientation.w
            prod = Product.Product(name, pose)
            self.products.append(prod)



        

