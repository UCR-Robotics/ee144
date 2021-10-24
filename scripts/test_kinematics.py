#!/usr/bin/env python

import rospy
import numpy as np
from math import pi
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from forward_kinematics import forward_kinematics
from inverse_kinematics import inverse_kinematics

class Manipulator():
    def __init__(self):
        rospy.init_node('manipulator')
        rospy.loginfo("Press Ctrl + C to terminate")
        self.rate = rospy.Rate(10)

		## PLease fill in your code here
        
        
        # prepare joint message to publish
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.name = ['waist', 'shoulder', 'elbow', 'wrist_angle',\
            'wrist_rotate', 'gripper', 'left_finger', 'right_finger']
        joint_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.026, -0.026]
        
        # test cases
        angles = [pi/6, -pi/3, -pi/6]  # joint angle in radian
        pos = [0.02165, 0.01250, 0.29721]
        position = forward_kinematics(angles)
        joint_angle = inverse_kinematics(pos)
        print('Position of end-effector = ', position)
        print('Joint angles = ', joint_angle)
        
        while not rospy.is_shutdown():
            joint_msg.header.stamp = rospy.Time.now()
            joint_msg.position[0:3] = angles
            self.joint_pub.publish(joint_msg)
            self.rate.sleep()


if __name__ == '__main__':
    whatever = Manipulator()
