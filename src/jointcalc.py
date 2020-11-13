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
        self.joints1 = Float64MultiArray()
        self.joints2 = Float64MultiArray()
        self.jointangles1 = rospy.Subscriber("camera1/robot/joints_pos",Float64MultiArray, self.callback1)
        self.jointangles2 = rospy.Subscriber("camera2/robot/joints_pos",Float64MultiArray, self.callback2)

    def callback1(self, data):
        self.joints1=data

    def callback2(self, data):
        self.joints2=data

def jointcalc(angles1, angles2):
    l2 = angles1[1]
    l3 = -1 *(angles2[1])
    r=3.5
    l23vector = np.array([r*np.cos(l2)*np.cos(l3), r*np.sin(l3), r*np.sin(l2)*np.cos(l3)])
    l4 = #still need to figure this one out
    return np.array([l2,l3,l4])

def main(args):
    rospy.init_node('join_calculation', anonymous=True)
    rate = rospy.Rate(1)
    jang = joint_angles()
    while not rospy.is_shutdown():
        jointcalc(jang.joints1, jang.joints2)
        rate.sleep()
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
