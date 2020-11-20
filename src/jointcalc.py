#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from scipy.optimize import least_squares

class joint_angles:

    def __init__(self):
        self.spheres1=np.zeros((4,3))
        self.spheres2=np.zeros((4,3))
        self.jointsactual = np.array([0.0,0.0,0.0])
        self.jointsest = np.array([0.0,0.0,0.0])
        #self.jointsactual=np.array([])
        self.pos1 = rospy.Subscriber("camera1/robot/spheres_pos",Float64MultiArray, self.callback1)
        self.pos2 = rospy.Subscriber("camera2/robot/spheres_pos",Float64MultiArray, self.callback2)
        self.angleactual1 = rospy.Subscriber('/robot/joint2_position_controller/command', Float64, self.callback3)
        self.angleactual2 = rospy.Subscriber('/robot/joint3_position_controller/command', Float64, self.callback4)
        self.angleactual3 = rospy.Subscriber('/robot/joint4_position_controller/command', Float64, self.callback5)
        #self.joint2 = rospy.Subscriber("robot/joint2_position_controller/command",Float64MultiArray, self.callback2)
        
    def callback1(self, data):
        self.spheres1= np.reshape(data.data, (4,2))

    def callback2(self, data):
        self.spheres2= np.reshape(data.data, (4,2))

    def callback3(self, data):
        self.jointsactual[0] = data.data
        
    def callback4(self, data):
        self.jointsactual[1] = data.data
        
    def callback5(self, data):
        self.jointsactual[2] = data.data

    def xrot_e(self, theta, a, b):
        m = np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
        return np.sum(np.abs(m.dot(a)-b))
        
    def yrot_e(self, theta, a, b):
        m = np.array([[np.cos(theta),0,-np.sin(theta)],[0,1,0],[np.sin(theta),0,np.cos(theta)]])
        return np.sum(np.abs(m.dot(a)-b))
        
    def find_angle(self, a, b, axis):
        b = b-a
        a = a/np.linalg.norm(a)
        b = b/np.linalg.norm(b)

        if (axis == 'x'):
            return least_squares(self.xrot_e, [0.0], args = (a,b), bounds = (-np.pi/2, np.pi/2)).x
        elif (axis == 'y'):
            return least_squares(self.yrot_e, [0.0], args = (a,b), bounds = (-np.pi/2, np.pi/2)).x

    def points3d(self):
        yellow = np.array([self.spheres2[0][0],self.spheres1[0][0],np.mean([self.spheres2[0][1],self.spheres1[0][1]])])
        blue = np.array([self.spheres2[1][0],self.spheres1[1][0],np.mean([self.spheres2[1][1],self.spheres1[1][1]])])
        green = np.array([self.spheres2[2][0],self.spheres1[2][0],np.mean([self.spheres2[2][1],self.spheres1[2][1]])])
        red = np.array([self.spheres2[3][0],self.spheres1[3][0],np.mean([self.spheres2[3][1],self.spheres1[3][1]])])
        return np.array([yellow,blue,green,red])
    
    def jointcalc(self):
        points = self.points3d()
        l1 = 0
        l2 = self.find_angle(points[2], points[1], 'x').item()
        l3 = self.find_angle(points[2], points[1], 'y').item()
        l4 = self.find_angle(points[3], points[2], 'x').item()
        return np.array([l1,l2,l3,l4])

        '''
        l2 = angles1[1]
        l3 = -1 *(angles2[1])
        r=3.5
        l23vector = np.array([r*np.cos(l2)*np.cos(l3), r*np.sin(l3), r*np.sin(l2)*np.cos(l3)])
        alpha = angles1[2]
        beta = (-1 * angles2[2])
        l34vector = np.array([r*np.cos(alpha)*np.cos(beta), r*np.sin(beta), r*np.sin(alpha)*np.cos(beta)])
        cross = np.cross(l23vector,l34vector)
        v_norm = np.array([1,0,0])
        v_norm=v_norm/np.linalg.norm(v_norm)
        l4 = np.arctan2(np.dot(cross, v_norm),(np.dot(l23vector, l34vector)))
        return np.array([l2,l3,l4])
        '''

def main(args):
    rospy.init_node('joint_calculation', anonymous=True)
    rate = rospy.Rate(1)
    jang = joint_angles()
    i = 0
    while ((len(jang.spheres1)==0) or (len(jang.spheres2)==0)):
        rate.sleep()
    while not (rospy.is_shutdown() or (i>=50)):
        print(jang.jointsactual - jang.jointcalc()[1:4])
        rate.sleep()
        i+=1
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
