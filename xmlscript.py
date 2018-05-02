#!/usr/bin/env python

import xml.etree.ElementTree as ET
tree = ET.parse("test.xml")
root = tree.getroot()

root = root[0]

#print(root)
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
                #print(scale[0][0].tag)
                try:
                scaleInt= scale[0][0].text.split(" ")
                
                except:

                #poseInt = list(map(float, scaleInt))
                print(scaleInt)

            



def addCollisions(self, scene):
        tree = ET.parse("test.xml")
        root = tree.getroot()
        i = 0
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
                    i = i + 1
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
                            if(sacle[0].tag == "mesh"):
                                a=0
                                
                        except: