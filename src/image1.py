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


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    #self.joints_pub = rospy.Publisher("camera1/robot/joints_pos",Float64MultiArray, queue_size=10, latch=True)
    self.pos_pub = rospy.Publisher("camera1/robot/spheres_pos",Float64MultiArray, queue_size=10, latch=True)
    self.target_pub = rospy.Publisher("camera1/robot/target_pos",Float64MultiArray, queue_size=10, latch=True)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

  def detect_target(self,image):
      mask = cv2.inRange(image, (0, 40, 100), (100, 100, 255))
      return self.detect_chamfer(mask)

  def detect_chamfer(self,image):
      chamfer = cv2.imread('chamfer.png')
      chamfer = cv2.cvtColor(chamfer, cv2.COLOR_BGR2GRAY)
      match = cv2.matchTemplate(mask, chamfer, 1)
      return np.array(cv2.minMaxLoc(match)[2])
    

  def detect_red(self,image):
      mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      if(M['m00'] != 0):
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
      else:
        cx = 0
        cy = 0
      return np.array([cx, cy])
 

  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      if(M['m00'] != 0):
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
      else:
        cx = 0
        cy = 0 
      return np.array([cx, cy])


  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      if(M['m00'] != 0):
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
      else:
        cx = 0
        cy = 0 
      return np.array([cx, cy])

  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      if(M['m00'] != 0):
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
      else:
        cx = 0
        cy = 0 
      return np.array([cx, cy])


  def pixel2meter(self,image):
      circle1Pos = self.detect_yellow(image)
      circle2Pos = self.detect_blue(image)
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 2.5 / np.sqrt(dist)

  def detect_sphere_locations(self, image):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob 
    center = a * self.detect_yellow(image)
    circle1Pos = a * self.detect_blue(image) 
    circle2Pos = a * self.detect_green(image) 
    circle3Pos = a * self.detect_red(image)
    return np.array([center, circle1Pos, circle2Pos, circle3Pos])

  def detect_joints(self, image):
    a = self.pixel2meter(image)
    center = a * self.detect_yellow(image)

    blue_pos = a * self.detect_blue(image)
    blue_to_yellow = center - blue_pos
    # This multiplication is as a result of the change in frames
    blue_to_yellow = blue_to_yellow * np.array([-1, 1])

    green_pos = a * self.detect_green(image)
    green_to_yellow = center - green_pos
    green_to_yellow = green_to_yellow * np.array([-1, 1])

    red_pos = a * self.detect_red(image)
    red_to_yellow = center - red_pos
    red_to_yellow = red_to_yellow * np.array([-1, 1])

    return np.array([center, blue_to_yellow, green_to_yellow, red_to_yellow])

  # This function is for testing purposes
  def detect_end_effector(self, image):
    a = self.pixel2meter(image)
    end_effector = a * (self.detect_yellow(image) - self.detect_red(image))
    end_effector = end_effector * np.array([-1, 1])
    return end_effector

  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)


    #im1=cv2.imshow('window1', self.cv_image)
    #a = self.detect_joint_angles(self.cv_image)
    #b = self.detect_sphere_locations(self.cv_image)
    b = self.detect_joints(self.cv_image)
    print(b[3])
    c = self.detect_target(self.cv_image)
    c = (b[0] - (c * self.pixel2meter(self.cv_image)))  * np.array([-1, 1])
    cv2.waitKey(1)

    #self.joints = Float64MultiArray()
    self.spheres = Float64MultiArray()
    self.target = Float64MultiArray()
    #self.joints.data = a
    self.spheres.data = np.reshape(b,(8))
    self.target.data = c
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
      #self.joints_pub.publish(self.joints)
      self.pos_pub.publish(self.spheres)
      self.target_pub.publish(self.target)
    except CvBridgeError as e:
      print(e)
    rospy.sleep(1)

# call the class
def main(args):
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


