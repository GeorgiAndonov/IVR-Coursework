#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from scipy.optimize import least_squares

# Jointcalc takes what's published from image1.py and image2.py and computes the result
class joint_angles:

    def __init__(self):
        self.spheres1=np.zeros((4,3))
        self.spheres2=np.zeros((4,3))
        self.jointsactual = np.array([0.0,0.0,0.0,0.0])
        self.jointsest = np.array([0.0,0.0,0.0,0.0])
        self.target1 = np.array([0.0,0.0])
        self.target2 = np.array([0.0,0.0])
        #self.jointsactual=np.array([])
        self.pos1 = rospy.Subscriber("camera1/robot/spheres_pos",Float64MultiArray, self.callback1)
        self.pos2 = rospy.Subscriber("camera2/robot/spheres_pos",Float64MultiArray, self.callback2)
        self.angleactual0 = rospy.Subscriber('/robot/joint1_position_controller/command', Float64, self.callback3)
        self.angleactual1 = rospy.Subscriber('/robot/joint2_position_controller/command', Float64, self.callback4)
        self.angleactual2 = rospy.Subscriber('/robot/joint3_position_controller/command', Float64, self.callback5)
        self.angleactual3 = rospy.Subscriber('/robot/joint4_position_controller/command', Float64, self.callback6)
        self.targetpos1 = rospy.Subscriber("camera1/robot/target_pos",Float64MultiArray, self.callback7)
        self.targetpos2 = rospy.Subscriber("camera2/robot/target_pos",Float64MultiArray, self.callback8)
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
        self.jointsactual[0] = data.data
        
    def callback4(self, data):
        self.jointsactual[1] = data.data

    def callback5(self, data):
        self.jointsactual[2] = data.data

    def callback6(self, data):
        self.jointsactual[3] = data.data

    def callback7(self, data):
        self.target1= data.data

    def callback8(self, data):
        self.target2= data.data

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
            try:
                return least_squares(self.xrot_e, [0.0], args = (a,b,l), bounds = (-np.pi/2, np.pi/2)).x
            except:
                return np.array([0.0])
        elif (axis == 'y'):
            try:
                return least_squares(self.yrot_e, [0.0], args = (a,b), bounds = (-np.pi/2, np.pi/2)).x
            except:
                return np.array([0.0])

    # Get the points in 3D
    def points3d2(self):
        blue = np.array([0, 0, 2])

        if self.spheres2[2][0] == 0:
            x_green = 0
            y_green = self.spheres1[2][0]
            z_green = self.spheres1[2][1]

        if self.spheres1[2][0] == 0:
            x_green = self.spheres2[2][0]
            y_green = 0
            z_green = self.spheres2[2][1]

        if self.spheres1[2][0] != 0 and self.spheres2[2][0] != 0:
            x_green = self.spheres2[2][0]
            y_green = self.spheres1[1][0]
            z_green = np.mean([self.spheres2[2][1], self.spheres1[2][1]])

        ee_pos = self.detect_end_effector(x_green, y_green)

        return np.array([np.array([0, 0, 0]), blue, np.array([x_green, y_green, z_green]), ee_pos])

    def detect_end_effector(self, x_green, y_green):
        if self.spheres2[3][0] == 0:
            x_ee = x_green
            y_ee = self.spheres1[3][0]
            z_ee = self.spheres1[3][1]

        if self.spheres1[3][0] == 0:
            x_ee = self.spheres2[3][0]
            y_ee = y_green
            z_ee = self.spheres2[3][1]

        if self.spheres2[3][0] != 0 and self.spheres1[3][0] != 0:
            x_ee = self.spheres2[3][0]
            y_ee = self.spheres1[3][0]
            z_ee = np.mean([self.spheres2[3][1], self.spheres1[3][1]])

        return np.array([x_ee, y_ee, z_ee])

    def target3d(self):
        return np.array([self.target2[0],self.target1[0], (self.target1[1] + self.target2[1]) / 2])
    
    def jointcalc(self):
        points = self.points3d2()
        l1 = 0
        
        l2 = self.find_angle(points[2], points[1], 3.5, 'x').item()
        l3 = self.find_angle(points[2], points[1], 0, 'y').item()
        l4 = self.find_angle(points[3], points[2], 3, 'x').item()
        return np.array([l1,l2,l3,l4])

    # Robot Control - part 3.1
    def forward_kinematics(self):
        angles = self.jointsactual
        cos_angle1, sin_angle1 = np.cos(angles[0] + np.pi/2), np.sin(angles[0] + np.pi/2)
        cos_angle2, sin_angle2 = np.cos(angles[1] + np.pi/2), np.sin(angles[1] + np.pi/2)
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

    # Jacobian
    def calculate_jacobian2(self, q):
        angles = q
        cos_angle1, sin_angle1 = np.cos(angles[0]), np.sin(angles[0])
        cos_angle2, sin_angle2 = np.cos(angles[1]), np.sin(angles[1])
        cos_angle3, sin_angle3 = np.cos(angles[2]), np.sin(angles[2])
        cos_angle4, sin_angle4 = np.cos(angles[3]), np.sin(angles[3])

        jac11 = 3.5 * cos_angle3 * cos_angle1 * sin_angle2 + 3.0 * sin_angle4 * cos_angle1 * cos_angle2 + 3 * cos_angle4 * \
              (cos_angle3 * cos_angle1 * sin_angle2 - sin_angle3 * sin_angle1) - 3.5 * sin_angle3 * sin_angle1

        jac12 = 3.5 * cos_angle3 * sin_angle1 * cos_angle2 - 3.0 * sin_angle4 * sin_angle1 * sin_angle2 + \
              3.0 * cos_angle4 * cos_angle3 * sin_angle1 * cos_angle2

        jac13 = -3.5 * sin_angle3 * sin_angle1 * sin_angle2 + 3.0 * cos_angle4 * \
              (-sin_angle3 * sin_angle1 * sin_angle2 + cos_angle3 * cos_angle1) + 3.5 * cos_angle3 * cos_angle1

        jac14 = 3.0 * cos_angle4 * sin_angle1 * cos_angle2 - 3.0 * sin_angle4 * \
              (cos_angle3 * sin_angle1 * sin_angle2 + sin_angle3 * cos_angle1)

        jac21 = 3.5 * cos_angle3 * sin_angle1 * sin_angle2 + 3.0 * sin_angle4 * sin_angle1 * cos_angle2 + 3.0 * \
            cos_angle4 * cos_angle3 * sin_angle2 * sin_angle1 + 3.0 * cos_angle4 * sin_angle3 * cos_angle1 + 3.5 * \
            sin_angle3 * cos_angle1

        jac22 = - 3.5 * cos_angle3 * cos_angle1 * cos_angle2 + 3.0 * sin_angle4 * cos_angle1 * sin_angle2 - 3.0 * \
            cos_angle4 * cos_angle3 * cos_angle2 * cos_angle1

        jac23 = 3.5 * sin_angle3 * cos_angle1 * sin_angle2 + 3.0 * cos_angle4 * sin_angle3 * sin_angle2 * \
            cos_angle1 + 3.0 * cos_angle4 * cos_angle3 * sin_angle1 + 3.5 * cos_angle3 * sin_angle1

        jac24 = -3.0 * cos_angle4 * cos_angle1 * cos_angle2 + 3.0 * cos_angle1 * sin_angle2 * cos_angle3 * \
            sin_angle4 - 3.0 * sin_angle1 * sin_angle3 * sin_angle4

        jac31 = 0

        jac32 = -3.0 * sin_angle2 * cos_angle3 * cos_angle4 - 3.5 * sin_angle2 * cos_angle3 - 3.0 * sin_angle4 * cos_angle2

        jac33 = -3.0 * sin_angle3 * cos_angle4 * cos_angle2 - 3.5 * sin_angle3 * cos_angle2

        jac34 = -3.0 * cos_angle2 * cos_angle3 * sin_angle4 - 3.0 * cos_angle4 * sin_angle2

        return np.array([[jac11, jac12, jac13, jac14],
                         [jac21, jac22, jac23, jac24],
                         [jac31, jac32, jac33, jac34]])

    def closed_loop_control(self):
        k_p = np.array([[0.5, 0, 0],
                        [0, 0.5, 0],
                        [0, 0, 0.5]])

        k_d = np.array([[0.005, 0, 0],
                        [0, 0.005, 0],
                        [0, 0, 0.005]])

        # estimate time step
        cur_time = np.array([rospy.get_time()])
        print("Current time: ")
        print(cur_time)
        print("Previous time:")
        print(self.time_previous_step)
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time

        ee_pos = self.points3d2()

        # get the end-effector position
        pos = ee_pos[3]  # Detect end-effector there
        # desired trajectory
        pos_d = self.target3d()  # Detect sphere coordinates
        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error) / dt
        # estimate error
        self.error = pos_d - pos
        q = self.jointsactual # Get the joint angles - jointcalc()
        J_inv = np.linalg.pinv(self.calculate_jacobian2(q))  # calculating the psudeo inverse of Jacobian
        dq_d = np.dot(J_inv, (np.dot(k_d, self.error_d.transpose()) + np.dot(k_p, self.error.transpose())))
        q_d = q + (dt * dq_d)  # control input (angular position of joints)
        return q_d


def closed_loop_test(jang, joint1pub, joint2pub, joint3pub, joint4pub, target_x, target_y, target_z, ee_x, ee_y, ee_z):
    target_coord = jang.target3d()
    q_d = jang.closed_loop_control()

    joint1 = Float64()
    joint1.data = q_d[0]

    joint2 = Float64()
    joint2.data = q_d[1]

    joint3 = Float64()
    joint3.data = q_d[2]

    joint4 = Float64()
    joint4.data = q_d[3]

    joint1pub.publish(joint1)
    joint2pub.publish(joint2)
    joint3pub.publish(joint3)
    joint4pub.publish(joint4)

    points = jang.points3d2()
    ee = points[3]

    ee_x.publish(ee[0])
    ee_y.publish(ee[1])
    ee_z.publish(ee[2])

    target_x.publish(target_coord[0])
    target_y.publish(target_coord[1])
    target_z.publish(target_coord[2])

# We have 2 main functions - the first one is for testing the FK and Estimation
# and the second one is for testing the control - uncomment the one you want to see

# def main(args):
#     rospy.init_node('joint_calculation', anonymous=True)
#     rate = rospy.Rate(3)
#     jang = joint_angles()
#     while not rospy.is_shutdown():
#         print("Forward kinematics: ")
#         print(jang.forward_kinematics())
#         print("EE estimation: ")
#         results = jang.points3d2()
#         print(results[3])
#         rate.sleep()
#     print("Shut down")


def main(args):
    rospy.init_node('joint_calculation', anonymous=True)
    rate = rospy.Rate(1)
    jang = joint_angles()
    joint1pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=10, latch=True)
    joint2pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=10, latch=True)
    joint3pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=10, latch=True)
    joint4pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=10, latch=True)
    target_x = rospy.Publisher('/target_x', Float64, queue_size=10, latch=True)
    target_y = rospy.Publisher('/target_y', Float64, queue_size=10, latch=True)
    target_z = rospy.Publisher('/target_z', Float64, queue_size=10, latch=True)
    ee_x = rospy.Publisher('/ee_x', Float64, queue_size=10, latch=True)
    ee_y = rospy.Publisher('/ee_y', Float64, queue_size=10, latch=True)
    ee_z = rospy.Publisher('/ee_z', Float64, queue_size=10, latch=True)

    print(jang.points3d2())
    while ((len(jang.spheres1)==0) or (len(jang.spheres2)==0)):
        rate.sleep()
    while not (rospy.is_shutdown()):
        closed_loop_test(jang, joint1pub, joint2pub, joint3pub, joint4pub, target_x, target_y, target_z, ee_x, ee_y, ee_z)
        rate.sleep()
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
