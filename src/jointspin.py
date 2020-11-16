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

def jointspins(joint2pub,joint3pub,joint4pub,zero_time):
  now = rospy.Time.now()
  timedur = (now - zero_time)
  time = timedur.to_sec()
  joint2pub.publish(np.pi/2 * np.sin(np.pi/15 * time))
  joint3pub.publish(np.pi/2 * np.sin(np.pi/18 * time))
  joint4pub.publish(np.pi/2 * np.sin(np.pi/20 * time))

def main(args):
  rospy.init_node('joint_rotation', anonymous=True)
  rate = rospy.Rate(1)
  joint2pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=10,latch=True)
  joint3pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=10, latch=True)
  joint4pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=10, latch=True)
  zero_time = rospy.Time()
  while not rospy.is_shutdown():
    jointspins(joint2pub,joint3pub,joint4pub,zero_time)
    rate.sleep
  print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
