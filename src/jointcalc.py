#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class joint_angles:

    def __init__(self):
        self.joints1=np.array([])
        self.joints2=np.array([])
        #self.jointsactual=np.array([])
        self.jointangles1 = rospy.Subscriber("camera1/robot/joints_pos",Float64MultiArray, self.callback1)
        self.jointangles2 = rospy.Subscriber("camera2/robot/joints_pos",Float64MultiArray, self.callback2)
        #self.joint2 = rospy.Subscriber("robot/joint2_position_controller/command",Float64MultiArray, self.callback2)
        
    def callback1(self, data):
        self.joints1=data.data

    def callback2(self, data):
        self.joints2=data.data

def jointcalc(angles1, angles2):
    l2 = angles1[1]
    l3 = -1 *(angles2[1])
    r=3.5
    l23vector = np.array([r*np.cos(l2)*np.cos(l3), r*np.sin(l3), r*np.sin(l2)*np.cos(l3)])
    alpha = angles1[2]
    beta = -1 * angles2[2]
    l34vector = np.array([r*np.cos(alpha)*np.cos(beta), r*np.sin(beta), r*np.sin(alpha)*np.cos(beta)])
    l4 = np.arccos((np.dot(l23vector, l34vector)/(np.linalg.norm(l23vector)*np.linalg.norm(l34vector))))
    cross = np.cross(l23vector,l34vector)
    v_norm = np.array([0,1,0])
    if (np.dot(v_norm,cross)<0):
        l4 = -l4
    return np.array([l2,l3,l4])

def main(args):
    rospy.init_node('join_calculation', anonymous=True)
    rate = rospy.Rate(1)
    jang = joint_angles()
    while (len(jang.joints1)==0):
        rate.sleep()
    while not rospy.is_shutdown():
        print(jointcalc(jang.joints1,jang.joints2))
        rate.sleep()
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
