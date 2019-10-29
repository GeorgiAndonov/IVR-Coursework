#!/usr/bin/env python


import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64


# Publish data
def move():
  rospy.init_node('target_pos_cmd', anonymous=True)
  rate = rospy.Rate(30) # 30hz
  # initialize a publisher to send joints' angular position to the robot
  robot_joint1_pub = rospy.Publisher("/target/x_position_controller/command", Float64, queue_size=10)
  robot_joint2_pub = rospy.Publisher("/target/y_position_controller/command", Float64, queue_size=10)
  robot_joint3_pub = rospy.Publisher("/target/z_position_controller/command", Float64, queue_size=10)
  t0 = rospy.get_time()
  while not rospy.is_shutdown():
    cur_time = np.array([rospy.get_time()])-t0
    #y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
    x_d = 1.5* np.cos(cur_time * np.pi/5)
    y_d = 1.5* np.sin(cur_time * np.pi/5)
    z_d = 1* np.sin(cur_time * np.pi/5)
    joint1=Float64()
    joint1.data= -1 + x_d
    joint2=Float64()
    joint2.data= -0.5 + y_d
    joint3=Float64()
    joint3.data= 5.5 + z_d
    robot_joint1_pub.publish(joint1)
    robot_joint2_pub.publish(joint2)
    robot_joint3_pub.publish(joint3)
    rate.sleep()



# run the code if the node is called
if __name__ == '__main__':
  try:
    move()
  except rospy.ROSInterruptException:
    pass


