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

def jointcalc():

def main(args):
    rospy.init_node('join_calculation', anonymous=True)
    rate = rospy.Rate(1)
    jointangles1 = rospy.Subscriber("camera1/robot/joints_pos",Float64MultiArray)
    jointangles2 = rospy.Subscriber("camera2/robot/joints_pos",Float64MultiArray)

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
