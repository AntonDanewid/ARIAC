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


class Subscriber: 

    def __init__(self):
        self.order_sub = rospy.Subscriber("/ariac/orders", Order, self.orderReceived)
        self.currentOrders = []


    def orderReceived(self, order):
        self.currentOrders.append(order)
        
