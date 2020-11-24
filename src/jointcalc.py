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
        self.jointsactual = np.array([0.0,0.0,0.0,0.0])
        self.jointsest = np.array([0.0,0.0,0.0,0.0])
        #self.jointsactual=np.array([])
        self.pos1 = rospy.Subscriber("camera1/robot/spheres_pos",Float64MultiArray, self.callback1)
        self.pos2 = rospy.Subscriber("camera2/robot/spheres_pos",Float64MultiArray, self.callback2)
        self.angleactual1 = rospy.Subscriber('/robot/joint2_position_controller/command', Float64, self.callback3)
        self.angleactual2 = rospy.Subscriber('/robot/joint3_position_controller/command', Float64, self.callback4)
        self.angleactual3 = rospy.Subscriber('/robot/joint4_position_controller/command', Float64, self.callback5)
        #self.joint2 = rospy.Subscriber("robot/joint2_position_controller/command",Float64MultiArray, self.callback2)
        self.time_trajectory = rospy.get_time()
        # initialize errors
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        # initialize error and derivative of error for trajectory tracking
        self.error = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0, 0.0], dtype='float64')
        #while not (rospy.is_shutdown():
                   

    def callback1(self, data):
        self.spheres1= np.reshape(data.data, (4,2))

    def callback2(self, data):
        self.spheres2= np.reshape(data.data, (4,2))

    def callback3(self, data):
        self.jointsactual[1] = data.data
        
    def callback4(self, data):
        self.jointsactual[2] = data.data
        
    def callback5(self, data):
        self.jointsactual[3] = data.data

    def translate(self, a,x,y,z):
        a_1 = np.concatenate((a, [1]))
        m = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[x,y,z,1]])
        a_2 = m.dot(a_1)
        return a_2[0:3]/a_2[3]

    def xrot_e(self, theta, a, b, l):
        #a_2 = self.translate(a,0,0,l)
        m = np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
        return np.sum(np.abs(m.dot(a)-b))
        
    def yrot_e(self, theta, a, b):
        #m =
        m = np.array([[np.cos(theta),0,-np.sin(theta)],[0,1,0],[np.sin(theta),0,np.cos(theta)]])
        return np.sum(np.abs(m.dot(a)-b))
        
    def find_angle(self, a, b, l, axis):
        a = a-b
        a = a/np.linalg.norm(a)
        b = b/np.linalg.norm(b)

        if (axis == 'x'):
            return least_squares(self.xrot_e, [0.0], args = (a,b,l), bounds = (-np.pi/2, np.pi/2)).x
        elif (axis == 'y'):
            return least_squares(self.yrot_e, [0.0], args = (a,b), bounds = (-np.pi/2, np.pi/2)).x

    def points3d(self):
        yellow = np.array([self.spheres2[0][0],self.spheres1[0][0],np.mean([self.spheres2[0][1],self.spheres1[0][1]])])
        blue = np.array([self.spheres2[1][0],self.spheres1[1][0],np.mean([self.spheres2[1][1],self.spheres1[1][1]])])
        green = np.array([self.spheres2[2][0],self.spheres1[2][0],np.mean([self.spheres2[2][1],self.spheres1[2][1]])])
        red = np.array([self.spheres2[3][0],self.spheres1[3][0],np.mean([self.spheres2[3][1],self.spheres1[3][1]])])
        #return np.array([yellow,blue,green,red])
        return np.array([yellow-yellow,yellow-blue,yellow-green,yellow-red])
    
    def jointcalc(self):
        points = self.points3d()
        l1 = 0
        
        l2 = self.find_angle(points[2], points[1], 3.5, 'x').item()
        l3 = self.find_angle(points[2], points[1], 0, 'y').item()
        l4 = self.find_angle(points[3], points[2], 3, 'x').item()
        return np.array([l1,l2,l3,l4])

    def target_coordinates(self):
        return 1

    # Robot Control - move to jointcalc
    def forward_kinematics(self):
        angles = self.jointcalc()
        cos_angle1, sin_angle1 = np.cos(angles[0] + 90), np.sin(angles[0] + 90)
        cos_angle2, sin_angle2 = np.cos(angles[1] + 90), np.sin(angles[1] + 90)
        cos_angle3, sin_angle3 = np.cos(angles[2]), np.sin(angles[2])
        cos_angle4, sin_angle4 = np.cos(angles[3]), np.sin(angles[3])

        x_e = 3.5 * cos_angle3 * cos_angle1 * cos_angle2 - 3 * sin_angle4 * cos_angle1 * sin_angle2 + \
              3 * cos_angle4 * (cos_angle3 * cos_angle1 * cos_angle2 + sin_angle3 * sin_angle1) + \
              3.5 * sin_angle3 * sin_angle1

        y_e = 3.5 * cos_angle3 * sin_angle1 * cos_angle2 - 3 * sin_angle4 * sin_angle1 * sin_angle2 + \
              3 * cos_angle4 * (cos_angle3 * sin_angle1 * cos_angle2 - sin_angle3 * cos_angle1) - \
              3.5 * sin_angle3 * cos_angle1

        z_e = 3 * cos_angle3 * cos_angle4 * sin_angle2 + 3.5 * cos_angle3 * sin_angle2 + 3 * sin_angle4 * cos_angle2 + 2.5

        end_effector = np.array([x_e, y_e, z_e])
        return end_effector

    # Calculate the robot Jacobian
    def calculate_jacobian(self):
        angles = self.jointcalc()
        cos_angle1, sin_angle1 = np.cos(angles[0] + 90), np.sin(angles[0] + 90)
        cos_angle2, sin_angle2 = np.cos(angles[1] + 90), np.sin(angles[1] + 90)
        cos_angle3, sin_angle3 = np.cos(angles[2]), np.sin(angles[2])
        cos_angle4, sin_angle4 = np.cos(angles[3]), np.sin(angles[3])

        dk_1 = [
            -3.5 * cos_angle3 * sin_angle1 * cos_angle2 + 3 * sin_angle4 * sin_angle1 * sin_angle2 + 3 * cos_angle4 *
            (-cos_angle3 * cos_angle2 * sin_angle1 + sin_angle3 * cos_angle1) + 3.5 * sin_angle3 * cos_angle1,
            -3.5 * cos_angle3 * cos_angle1 * sin_angle2 - 3 * sin_angle4 * cos_angle1 * cos_angle2 - 3 * cos_angle4 *
            cos_angle3 * cos_angle1 * sin_angle2,
            -3.5 * sin_angle3 * cos_angle1 * cos_angle2 + 3 * cos_angle4(-sin_angle3 * cos_angle1 * cos_angle2 +
                                                                         cos_angle3 * sin_angle1) + 3.5 * cos_angle3 * sin_angle1,
            -3 * cos_angle4 * cos_angle1 * sin_angle2 - 3 * sin_angle4 * (
                    cos_angle3 * cos_angle1 * cos_angle2 + sin_angle3 * sin_angle1)]

        dk_2 = [3.5 * cos_angle3 * cos_angle1 * cos_angle2 - 3 * sin_angle4 * cos_angle1 * sin_angle2 + 3 * cos_angle4 *
                (cos_angle3 * cos_angle2 * cos_angle1 + sin_angle3 * sin_angle1) + 3.5 * sin_angle3 * sin_angle1,
                -3.5 * cos_angle3 * sin_angle1 * sin_angle2 - 3 * sin_angle4 * sin_angle1 * cos_angle2 - 3 * cos_angle4 *
                cos_angle3 * sin_angle1 * sin_angle2,
                -3.5 * sin_angle3 * sin_angle1 * cos_angle2 + 3 * cos_angle4(-sin_angle3 * sin_angle1 * cos_angle2 -
                                                                             cos_angle3 * cos_angle1) - 3.5 * cos_angle3 * cos_angle1,
                -3 * cos_angle4 * sin_angle1 * sin_angle2 - 3 * sin_angle4 * (
                        cos_angle3 * sin_angle1 * cos_angle2 - sin_angle3 * cos_angle1)]

        dk_3 = [0,
                3 * cos_angle3 * cos_angle4 * cos_angle2 + 3.5 * cos_angle3 * cos_angle2 - 3 * sin_angle4 * sin_angle2,
                -3 * sin_angle3 * cos_angle4 * sin_angle2 - 3.5 * sin_angle3 * sin_angle2,
                -3 * cos_angle3 * sin_angle4 * sin_angle2 + 3 * cos_angle4 * cos_angle2]

        return np.array([dk_1,
                         dk_2,
                         dk_3])

    def closed_loop_control(self):
        k_p = np.array([[0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0]])

        k_d = np.array([[0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0]])

        # estimate time step
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time

        circle_pos = self.points3d()

        # get the end-effector position
        pos = circle_pos[3]  # Detect end-effector there
        # desired trajectory
        pos_d = self.target_coordinates()  # Detect sphere coordinates
        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error) / dt
        # estimate error
        self.error = pos_d - pos
        q = self.jointcalc() # Get the joint angles - jointcalc()
        J_inv = np.linalg.pinv(self.calculate_jacobian(q))  # calculating the psudeo inverse of Jacobian
        dq_d = np.dot(J_inv, (np.dot(k_d, self.error_d.transpose()) + np.dot(k_p, self.error.transpose())))
        q_d = q + (dt * dq_d)  # control input (angular position of joints)
        return q_d


def main(args):
    rospy.init_node('joint_calculation', anonymous=True)
    rate = rospy.Rate(1)
    jang = joint_angles()
    i = 0
    angles = []
    while ((len(jang.spheres1)==0) or (len(jang.spheres2)==0)):
        rate.sleep()
    while not (rospy.is_shutdown() or (i>=50)):
        angles.append([jang.jointsactual[1], jang.jointcalc()[1]])
        rate.sleep()
        i+=1
    plt.plot(angles)
    plt.show()
    rospy.spin()
    #print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
