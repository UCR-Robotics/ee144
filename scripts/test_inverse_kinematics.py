#!/usr/bin/env python

import rospy
import numpy as np
from math import pi
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from inverse_kinematics import inverse_kinematics

class Manipulator():
    def __init__(self):
        rospy.init_node('manipulator')
        rospy.loginfo("Press Ctrl + C to terminate")
        self.rate = rospy.Rate(10)
        self.joint_pub = rospy.Publisher('/rx150/joint_states', JointState, queue_size=10)
        
        # prepare joint message to publish
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.name = ['waist', 'shoulder', 'elbow', 'wrist_angle',\
            'wrist_rotate', 'gripper', 'left_finger', 'right_finger']
        joint_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.026, -0.026]
        
        # test case for inverse kinematics
        test_case = [0.02165, 0.01250, 0.29721]  # position (x, y, z) in meter
        joint_angle = inverse_kinematics(test_case)
        print(test_case)
        print(joint_angle)
        
        while not rospy.is_shutdown():
            joint_msg.header.stamp = rospy.Time.now()
            joint_msg.position[0:3] = joint_angle
            self.joint_pub.publish(joint_msg)
            self.rate.sleep()


if __name__ == '__main__':
    whatever = Manipulator()
