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
    self.joints_pub = rospy.Publisher("camera1/robot/joints_pos",Float64MultiArray, queue_size=10, latch=True)
    self.pos_pub = rospy.Publisher("camera1/robot/spheres_pos",Float64MultiArray, queue_size=10, latch=True)
    self.target_pub = rospy.Publisher("camera1/robot/target_pos",Float64MultiArray, queue_size=10, latch=True)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

  def detect_target(self,image):
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, (25, 80, 30), (35, 100, 90))
      #bgr = cv2.cvtColor(mask, cv2.COLOR_HSV2BGR)
      #gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
      circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1.2, 100)
      if circles is not None:
        return circles[0,0][0:2]
      #else:
      #  raise ValueError

  # In this method you can focus on detecting the centre of the red circle
  def detect_red(self,image):      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      if(M['m00'] != 0):
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
      else:
        #If the red circle is occluded, try the green circle
        mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if(M['m00'] != 0):
          cx = int(M['m10'] / M['m00'])
          cy = int(M['m01'] / M['m00'])
        else:
          #if the green circle is occluded, try the blue circle
          mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
          kernel = np.ones((5, 5), np.uint8)
          mask = cv2.dilate(mask, kernel, iterations=3)
          M = cv2.moments(mask)
          cx = int(M['m10'] / M['m00'])
          cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])
 

  # Detecting the centre of the green circle
  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      if(M['m00'] != 0):
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
      else:
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if(M['m00'] != 0):
          cx = int(M['m10'] / M['m00'])
          cy = int(M['m01'] / M['m00'])
        else:
          mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
          kernel = np.ones((5, 5), np.uint8)
          mask = cv2.dilate(mask, kernel, iterations=3)
          M = cv2.moments(mask)
          cx = int(M['m10'] / M['m00'])
          cy = int(M['m01'] / M['m00'])   
      return np.array([cx, cy])


  # Detecting the centre of the blue circle
  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  # Detecting the centre of the yellow circle
  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  # Calculate the conversion from pixel to meter
  def pixel2meter(self,image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_blue(image)
      circle2Pos = self.detect_green(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 3 / np.sqrt(dist)


  # Calculate the relevant joint angles from the image
  def detect_joint_angles(self,image):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob 
    center = a * self.detect_yellow(image)
    circle1Pos = a * self.detect_blue(image) 
    circle2Pos = a * self.detect_green(image) 
    circle3Pos = a * self.detect_red(image)
    # Solve using trigonometry
    ja1 = np.arctan2(center[0]- circle1Pos[0], center[1] - circle1Pos[1])
    ja2 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1
    ja3 = np.arctan2(circle2Pos[0]-circle3Pos[0], circle2Pos[1]-circle3Pos[1]) - ja2 - ja1
    return np.array([ja1, ja2, ja3])

  def detect_sphere_locations(self, image):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob 
    center = a * self.detect_yellow(image)
    circle1Pos = a * self.detect_blue(image) 
    circle2Pos = a * self.detect_green(image) 
    circle3Pos = a * self.detect_red(image)
    return np.array([center, circle1Pos, circle2Pos, circle3Pos])

  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)


    im1=cv2.imshow('window1', self.cv_image)
    a = self.detect_joint_angles(self.cv_image)
    b = self.detect_sphere_locations(self.cv_image)
    c = self.detect_target(self.cv_image)
    cv2.waitKey(1)

    self.joints = Float64MultiArray()
    self.spheres = Float64MultiArray()
    self.target = Float64MultiArray()
    self.joints.data = a
    self.spheres.data = b
    self.target.data = c
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
      self.joints_pub.publish(self.joints)
      self.pos_pub.publish(self.spheres)
      self.target_pub.publish(self.target)
    except CvBridgeError as e:
      print(e)

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


