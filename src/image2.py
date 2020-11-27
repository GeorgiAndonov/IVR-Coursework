#!/usr/bin/env python3
# N

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
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,callback=self.callback2)
    #self.joints_pub = rospy.Publisher("camera2/robot/joints_pos",Float64MultiArray, queue_size=10, latch=True)
    self.pos_pub = rospy.Publisher("camera2/robot/spheres_pos",Float64MultiArray, queue_size=10, latch=True)
    self.target_pub = rospy.Publisher("camera2/robot/target_pos",Float64MultiArray, queue_size=10, latch=True)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

  # Detect target
  def detect_target(self,image):
      mask = cv2.inRange(image, (0, 40, 100), (100, 100, 255))
      return self.detect_chamfer(mask)

  def detect_chamfer(self,image):
      chamfer = cv2.imread('src/ivr_assignment/src/chamfer.png')
      chamfer = cv2.cvtColor(chamfer, cv2.COLOR_BGR2GRAY)
      match = cv2.matchTemplate(image, chamfer, cv2.TM_CCOEFF_NORMED)
      return np.array(cv2.minMaxLoc(match)[2])

  # Detect joints
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


  # Calculate the conversion from pixel to meter
  def pixel2meter(self,image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_yellow(image)
      circle2Pos = self.detect_blue(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 2.5 / np.sqrt(dist)

  # Test function
  def detect_sphere_locations(self, image):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob 
    center = a * self.detect_yellow(image)
    circle1Pos = a * self.detect_blue(image) 
    circle2Pos = a * self.detect_green(image) 
    circle3Pos = a * self.detect_red(image)
    return np.array([center, circle1Pos, circle2Pos, circle3Pos])

  # Get the joints
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

    red_to_yellow = self.detect_end_effector(image)

    return np.array([center, blue_to_yellow, green_to_yellow, red_to_yellow])

  def detect_end_effector(self, image):
    a = self.pixel2meter(image)
    red_coord = self.detect_red(image)
    # In case it is not visible
    if red_coord[0] == 0 and red_coord[1] == 0:
        return red_coord
    else:
        end_effector = a * (self.detect_yellow(image) - red_coord)
        return end_effector * np.array([-1, 1])

  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    #im2=cv2.imshow('window2', self.cv_image)
    #a = self.detect_joint_angles(self.cv_image)
    #b = self.detect_sphere_locations(self.cv_image)

    # Get and publish the results
    b = self.detect_joints(self.cv_image)
    print(b[3])
    c = self.detect_target(self.cv_image)
    c = (b[0] - (c.astype(float) * self.pixel2meter(self.cv_image)))
    cv2.waitKey(1)

    #self.joints = Float64MultiArray()
    self.spheres = Float64MultiArray()
    self.target = Float64MultiArray()
    #self.joints.data = a
    self.spheres.data = np.reshape(b,(8))
    self.target.data = c

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
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


