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


import dynamic_reconfigure.client








# def start_competition():
#     rospy.loginfo("Waiting for competition to be ready...")
#     rospy.wait_for_service('/ariac/start_competition')
#     rospy.loginfo("Competition is now ready.")
#     rospy.loginfo("Requesting competition start...")

#     try:
#         start = rospy.ServiceProxy('/ariac/start_competition', Trigger)
#         response = start()
#     except rospy.ServiceException as exc:
#         rospy.logerr("Failed to start the competition: %s" % exc)
#         return False
#     if not response.success:
#         rospy.logerr("Failed to start the competition: %s" % response)
#     else:
#         rospy.loginfo("Competition started!")
#     return response.success


def control_gripper(enabled):
    rospy.loginfo("Waiting for gripper control to be ready...")
    rospy.wait_for_service('/ariac/gripper/control')
    rospy.loginfo("Gripper control is now ready.")
    rospy.loginfo("Requesting gripper control...")

    try:
        gripper_control = rospy.ServiceProxy('/ariac/gripper/control', VacuumGripperControl)
        response = gripper_control(enabled)
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to control the gripper: %s" % exc)
        return False
    if not response.success:
        rospy.logerr("Failed to control the gripper: %s" % response)
    else:
        rospy.loginfo("Gripper controlled successfully")
    return response.success





class ArmControll:
    def __init__(self):
        self.joint_trajectory_publisher = \
            rospy.Publisher("/ariac/arm/command", JointTrajectory, queue_size=10)
        self.current_comp_state = None
        self.received_orders = []
        self.current_joint_state = None
        self.current_gripper_state = None
        self.last_joint_state_print = time.time()
        self.last_gripper_state_print = time.time()
        self.has_been_zeroed = False
        self.arm_joint_names = [
            'iiwa_joint_1',
            'iiwa_joint_2',
            'iiwa_joint_3',
            'iiwa_joint_4',
            'iiwa_joint_5',
            'iiwa_joint_6',
            'iiwa_joint_7',
            'linear_arm_actuator_joint'
        ]



        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander()
        rospy.sleep(2)

        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(2)
        #self.addCollisions(self.scene)
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)
        rospy.sleep(2)
        self.group.allow_replanning(True)
        #self.tf_listener = tf.TransformListener()

        # Allow 5 seconds per planning attempt
        self.group.set_planning_time(5)
        # Set goal joint tolerance
        self.group.set_goal_joint_tolerance(0.005)
        # Set goal goal tolerance
        self.group.set_goal_tolerance(0.005)
        # Set goal goal tolerance
        self.group.set_goal_position_tolerance(0.005)
        # Set goal orientation tolerance
        self.group.set_goal_orientation_tolerance(0.005)
        
        collision_object_pub = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject)
  
        self.tf_listener = tf.TransformListener()
    
        print("============ INITILIZED ARM LATESTE")
      
        

 
  
    def send_start(self):
        self.send_arm_to_state(bin3_init)
        rospy.sleep(2)
        self.send_arm_to_state(startbin)

 
        
        # box = PoseStamped()
        # box.header = self.robot.get_planning_frame()
        # box.pose.position.x = 0.27500
        # box.pose.position.y = -1
        # box.pose.position.z = 1.4
        
        # display_trajectory_publisher = rospy.Publisher(
        #                             '/move_group/display_planned_path',
        #                             moveit_msgs.msg.DisplayTrajectory)

        # rospy.loginfo(str(msg.shipments[0].products[0].pose.position.x))
        # pose_bin1_init = geometry_msgs.msg.Pose()

        # pose_bin1_init.position.x = -0.75
       

        # pose_bin1_init.position.y = 1.186137
        # #pose_bin1_init.position.y = -0.4

        # #                                      #roll?         rolll
        # pose_bin1_init.position.z = 1.186137
        # #q = quaternion_from_euler(0.064575, 0.329187, -3.060996)
        # q = quaternion_from_euler(0.329187, 0.064575 , -3.060996)

        # #pose_bin1_init.orientation.x = q[0]
        # #pose_bin1_init.orientation.y = q[1]
        # #pose_bin1_init.orientation.z = q[2]
        # #pose_bin1_init.orientation.w = q[3]
      
        # pose_start = geometry_msgs.msg.Pose()
        # pose_start.position.x = -0.05
        # pose_start.position.y = 1
        # pose_start.position.z = 2.01
        # pose_start.orientation.w = 1
        # self.group.set_pose_target(pose_bin1_init)
        # plan1 = self.group.plan()
      
        # self.group.set_joint_value_target(start)
        # plan1 = self.group.plan()
        # self.group.execute(plan1)
        # rospy.sleep(10)
    

    def gripper_state_callback(self, msg):
        if time.time() - self.last_gripper_state_print >= 10:
            rospy.loginfo("Current gripper state (throttled to 0.1 Hz):\n" + str(msg))
            self.last_gripper_state_print = time.time()
        self.current_gripper_state = msg

    def send_arm_to_state(self, positions):
        msg = JointTrajectory()
        msg.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(1.0)
        msg.points = [point]
        rospy.loginfo("Sending command:\n" + str(msg))

        test = {}
        test['iiwa_joint_1'] = positions[0]
        test['iiwa_joint_2'] = positions[1]
        test['iiwa_joint_3'] = positions[2]
        test['iiwa_joint_4'] = positions[3]
        test['iiwa_joint_5'] = positions[4]
        test['iiwa_joint_6'] = positions[5]
        test['iiwa_joint_7'] = positions[6]
        test['linear_arm_actuator_joint'] = positions[7]
        #self.joint_trajectory_publisher.publish(msg)
        #group_variable_values = self.group.get_current_joint_values()
        #\\group_variable_values = positions
        self.group.set_joint_value_target(test)
        self.group.plan()
        self.group.go(wait=True)



    def sendOverBin(self, bin): 
        if(bin == 2):
            self.send_arm_to_state(bin2_init)
            rospy.sleep(2)
            self.send_arm_to_state(bin2_hover)
            rospy.sleep(2)
        if(bin == 3):
            self.send_arm_to_state(bin3_init)
            rospy.sleep(2)
            self.send_arm_to_state(bin3_hover)
            rospy.sleep(2)
        if(bin == 4):
            self.send_arm_to_state(bin4_init)
            rospy.sleep(2)
            self.send_arm_to_state(bin4_hover)
            rospy.sleep(2)

    

    def sendBackFromBin(self, bin):
        if(bin == 2):
            self.send_arm_to_state(bin2_hover)
            rospy.sleep(2)
            self.send_arm_to_state(bin2_init)
            rospy.sleep(2)
        if(bin == 4): 
            self.send_arm_to_state(bin4_hover)
            rospy.sleep(2)
            self.send_arm_to_state(bin4_init)
            rospy.sleep(2)
        if(bin == 3):
            self.send_arm_to_state(bin3_hover)
            rospy.sleep(2)
            self.send_arm_to_state(bin3_init)
            rospy.sleep(2)
    
    
    def grabPart(self, productPose):
        workspace = [productPose.pose.position.x-0.5, productPose.pose.position.y -0.5, productPose.pose.position.z, productPose.pose.position.x + 0.5, productPose.pose.position.y +0.5, productPose.pose.position.z -0.1]
        self.group.set_workspace(workspace)
        xyz = [0, 0, 0]
        
        xyz[0] = productPose.pose.position.x 
        xyz[1] = productPose.pose.position.y
        xyz[2] = productPose.pose.position.z
        #self.group.set_position_target(xyz)

        target = geometry_msgs.msg.Pose()
        # target.position = productPose.pose.position
        # target.position.z = target.position.z +0.1
        
        #target.orientation.w = -1

        
        #quaternion = tf.transformations.quaternion_from_euler(0.331586, 0.068785, 1.452098)
        #type(pose) = geometry_msgs.msg.Pose
        #norm = quaternion[0]*quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] + quaternion[3]* quaternion[3]
        target = self.group.get_current_pose().pose
        print(target.orientation)
        target.position.x = productPose.pose.position.x
        target.position.y = productPose.pose.position.y
        target.position.z = productPose.pose.position.z


        target.position.z = target.position.z + 0.1
        # #norm = math.sqrt(norm)
        # target.orientation.x = quaternion[0] 
        # target.orientation.y = quaternion[1] 
        # target.orientation.z = quaternion[2] 
        # target.orientation.w = quaternion[3] 
        #self.group.set_pose_target(target)
        self.group.set_pose_target(target)
        self.group.plan()
        print("=================CURRENT STATE IS", self.robot.get_current_state)
        self.group.go(wait=True)
        rospy.sleep(2)
        control_gripper(True)




    def transformPose(self, pose, posOffset, orienOffset, frame):
            # Transform pose using frame
        targetPose = PoseStamped()
        targetPose.header.frame_id = frame
        targetPose.pose.position.x = pose.position.x + posOffset[0]
        targetPose.pose.position.y = pose.position.y + posOffset[1]
        targetPose.pose.position.z = pose.position.z + posOffset[2]
        targetPose.pose.orientation.x = 0.0 + orienOffset[0]
        targetPose.pose.orientation.y = 0.0 + orienOffset[1]
        targetPose.pose.orientation.z = 0.0 + orienOffset[2]
        targetPose.pose.orientation.w = 0.0 + orienOffset[3]
            # Transform


        transformedPose = self.tf_listener.transformPose('world', targetPose)
            # Return
        return transformedPose
        



    def poseTarget(self, target):
        # Given a target try and position arm end effector over it, return bool
        poseTarget = geometry_msgs.msg.Pose()
        poseTarget.position.x = target.pose.position.x
        poseTarget.position.y = target.pose.position.y
        poseTarget.position.z = target.pose.position.z
        poseTarget.orientation = target.pose.orientation
        self.group.set_pose_target(poseTarget)


        #self.group.set_pose_target(pose_target)
        #plan1 = self.group.plan()        


    def planPose(self):
        # Plan arm pose 
        plan = self.group.plan()

    def executePlan(self):
        # Execute desired plan
        self.group.go(wait=True)




    def sendOverTray(self):
        print("no")


    def addCollisions(self, scene):
            tree = ET.parse("test.xml")
            root = tree.getroot()
            i = 0
            collision_object_pub = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject)
            root = root[0]

            for neighbor in root:
                for neb in neighbor.iter('model'):        
                    for collision in neb.iter('collision'):
                        poseInt = []
                        scaleInt = []
                        for pose in collision.iter('pose'):
                            poseInt = pose.text.split(" ")
                            poseInt = list(map(float, poseInt))
                        #print(poseInt)
                        for scale in collision.iter('geometry'):
                            try:
                                scaleInt= scale[0][0].text.split(" ")
                                if(scale[0].tag == "box"):
                                
                                    box = PoseStamped()
                                    box.header = self.robot.get_planning_frame()
                                    box.pose.position.x = poseInt[0]
                                    box.pose.position.y = poseInt[1]
                                    box.pose.position.z = poseInt[2]
                                    print("Adding box")
                                    self.scene.add_box(str(i), box, scaleInt)
                                    self.scene.applyCollisionObject(box)
                                    
                                    collision_object = moveit_msgs.msg.CollisionObject()
                    
                                    i = i + 1


                                    print('===============================')
                                if(scale[0].tag == "mesh"):
                                    a = 0
                                if(scale[0].tag == "cylinder"):
                                    #Fix so that we approximate a box for the cylinder
                                    length = scale[0][0]
                                    radius = scale[0][1]
                                                            
                                    box = PoseStamped()
                                    box.header = self.robot.get_planning_frame()
                                    box.pose.position.x = poseInt[0]
                                    box.pose.position.y = poseInt[1]
                                    box.pose.position.z = poseInt[2]
                                    self.scene.add_box(str(i), box, (radius*2, radius*2, length))
                                    i = i + 1

                                if(sacle[0].tag == "mesh"):
                                    a=0
                                    
                            except:
                                pass





#bin3_init = [0.8842504439136532, 1.7269822672085962, 1.5322848955112205, -1.9922189839973683, -1.5687392233618356, 1.2100346569304605, -1.7403053744518617, -0.27145424520942707, 0.0]
bin3_init = [-1.780048118886426, 1.6693944180336215, 2.8248366387726787, -0.54709608097345, -0.12053863933896025, -1.9436729915697848, 1.3568785660073148, 0.8701813116885472, 0.0]
#bin3_hover = [2.030708303027902, 1.5310893282578615, 1.5749289822025245, -1.3416422593103556, -1.5923680850468767, 1.253505298922895, -1.1957209389989902, -0.18298717536680742, 0.0]
bin3_hover = [-2.5846364547017586, 1.8007230116584498, 2.9321449503515318, -0.3729635718692652, 0.09599146144110371, -1.5579043743516792, 0.6283811936378756, 0.5992819901439533, 0.0]
start = [0, 0, 0, 0 ,0 ,0 ,0, 0, 0]
startbin = [1.092518983885773, 1.4819056485211508, 1.6481364135030203, -1.989019987811874, -1.4185319842752095, 1.3006881728757307, -1.4940562409131344, -0.33563946341297035, 0.0]
bin2_hover = [2.1376512939425787, 1.577703386233594, 1.4965436365912836, -1.3189158894505644, -1.6483632779223516, 1.168788523744439, -1.0995023333076688, -0.8582111719843412, 0.0]
bin2_init = [1.1834538606529188, 1.606397393784972, 1.3479349307800135, -2.021097598382318, -1.7075385940188674, 1.0746057037765455, -1.354026514380564, -1.0339370040978124, 0.0]
bin4_init = [0.8468802479742044, 1.557319000511594, 1.5085020367811497, -1.9957541406818562, -1.6633651277989463, 1.1952518862140886, -1.9923196606145552, 0.476965360991872, 0.0]
bin4_hover = [2.2568991548044846, 1.5778967907433996, 1.5845747874402276, -1.1774160457018974, -1.6101804156501398, 1.3317601741295064, -1.4415122391816748, 0.7287825828358929, 0.0]
