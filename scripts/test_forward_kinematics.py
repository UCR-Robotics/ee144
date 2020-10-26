#!/usr/bin/env python

import rospy
import numpy as np
from math import pi
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from forward_kinematics import forward_kinematics

class Manipulator():
    def __init__(self):
        rospy.init_node('manipulator')
        self.rate = rospy.Rate(10)
        self.joint_pub = rospy.Publisher('/rx150/joint_states', JointState, queue_size=10)
        
        # prepare joint message to publish
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.name = ['waist', 'shoulder', 'elbow', 'wrist_angle',\
            'wrist_rotate', 'gripper', 'left_finger', 'right_finger']
        joint_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.026, -0.026]
        
        # test case for forward kinematics
        test_case = [pi/6, -pi/3, -pi/6]  # joint angle in radian
        position = forward_kinematics(test_case)
        print(test_case)
        print(position)
        
        for i in range(20):
            joint_msg.header.stamp = rospy.Time.now()
            joint_msg.position[0:3] = test_case
            self.joint_pub.publish(joint_msg)
            self.rate.sleep()


if __name__ == '__main__':
    whatever = Manipulator()
