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

            



