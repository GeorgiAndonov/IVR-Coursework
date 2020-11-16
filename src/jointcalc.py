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
        self.jointsactual = np.array([0.0,0.0,0.0])
        #self.jointsactual=np.array([])
        self.jointangles1 = rospy.Subscriber("camera1/robot/joints_pos",Float64MultiArray, self.callback1)
        self.jointangles2 = rospy.Subscriber("camera2/robot/joints_pos",Float64MultiArray, self.callback2)
        self.angleactual1 = rospy.Subscriber('/robot/joint2_position_controller/command', Float64, self.callback3)
        self.angleactual2 = rospy.Subscriber('/robot/joint3_position_controller/command', Float64, self.callback4)
        self.angleactual3 = rospy.Subscriber('/robot/joint4_position_controller/command', Float64, self.callback5)
        #self.joint2 = rospy.Subscriber("robot/joint2_position_controller/command",Float64MultiArray, self.callback2)
        
    def callback1(self, data):
        self.joints1=data.data

    def callback2(self, data):
        self.joints2=data.data

    def callback3(self, data):
        self.jointsactual[0] = data.data
        
    def callback4(self, data):
        self.jointsactual[1] = data.data
        
    def callback5(self, data):
        self.jointsactual[2] = data.data


def jointcalc(angles1, angles2):
    l2 = angles1[1]
    l3 = -1 *(angles2[1])
    r=3.5
    l23vector = np.array([r*np.cos(l2)*np.cos(l3), r*np.sin(l3), r*np.sin(l2)*np.cos(l3)])
    alpha = angles1[2]
    beta = -1 * angles2[2]
    l34vector = np.array([r*np.cos(alpha)*np.cos(beta), r*np.sin(beta), r*np.sin(alpha)*np.cos(beta)])
    cross = np.cross(l23vector,l34vector)
    v_norm = np.array([1,0,0])
    v_norm=v_norm/np.linalg.norm(v_norm)
    l4 = np.arctan2(np.dot(cross, v_norm),(np.dot(l23vector, l34vector)))
    return np.array([l2,l3,l4])

def main(args):
    rospy.init_node('join_calculation', anonymous=True)
    rate = rospy.Rate(1)
    jang = joint_angles()
    i = 0
    while ((len(jang.joints1)==0) or (len(jang.joints2)==0)):
        rate.sleep()
    while not (rospy.is_shutdown() or (i>=20)):
        print(jang.jointsactual - jointcalc(jang.joints1,jang.joints2))
        rate.sleep()
        i+=1
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
