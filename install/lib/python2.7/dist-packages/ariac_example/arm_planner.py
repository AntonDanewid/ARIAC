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







class ArmPlanner:
    #def __init__(self):

    def comp_state_callback(self, msg):
        if self.current_comp_state != msg.data:
            rospy.loginfo("Competition state: " + str(msg.data))
        self.current_comp_state = msg.data

    def order_callback(self, msg):
        rospy.loginfo("Received order:\n" + str(msg))
        self.received_orders.append(msg)
        control_conveyor(51)




    def joint_state_callback(self, msg):
        if time.time() - self.last_joint_state_print >= 10:
            rospy.loginfo("Current Joint States (throttled to 0.1 Hz):\n" + str(msg))
            self.last_joint_state_print = time.time()
        self.current_joint_state = msg







def connect_callbacks(planner):
    planner_subscriber = rospy.Subscriber(
        "/ariac/planner", String, planner.comp_state_callback)
