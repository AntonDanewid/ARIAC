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

from ariac_example import ariac_example
from std_srvs.srv import Trigger
from osrf_gear.msg import Order
import rospy



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

    def orderReceived(self, order):
        self.currentOrders.append(order)




def main():

    sub = Subscriber()
    armcontroll = ariac_example.ArmControll()
    #Start()
    rospy.loginfo("=============Setup complete.")
    armcontroll.sendOverBin(3)


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



