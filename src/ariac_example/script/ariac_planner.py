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
#THIS IS A REFERENCE msg.shipments[0].products[0].pose.position.x


class Planner:
    state = 0
    current_order = 0
    def recieve_order(self, msg):
        shipments = msg.shipments
        print(shipments)


    def start_competition(self):
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

    def Planner(self):
        start_competition()
        order_sub = rospy.Subscriber("/ariac/orders", Order, recieve_order)
