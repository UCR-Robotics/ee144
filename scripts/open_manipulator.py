#!/usr/bin/env python

import rospy
import numpy as np
from math import pi, cos, sin, atan2, sqrt, acos
from std_msgs.msg import Float64
from collections import namedtuple

position = namedtuple('position', ['x', 'y', 'z'])
jointangle = namedtuple('jointangle', ['theta1', 'theta2', 'theta3', 'theta4'])


class OpenManipulator():
    def __init__(self):
        rospy.init_node('open_manipulator_move', anonymous=False)
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.reset)
        self.rate = rospy.Rate(10)
        
        self.pub_joint1 = rospy.Publisher('/open_manipulator/joint1_position/command', Float64, queue_size=10)
        self.pub_joint2 = rospy.Publisher('/open_manipulator/joint2_position/command', Float64, queue_size=10)
        self.pub_joint3 = rospy.Publisher('/open_manipulator/joint3_position/command', Float64, queue_size=10)
        self.pub_joint4 = rospy.Publisher('/open_manipulator/joint4_position/command', Float64, queue_size=10)

        # test case for forward kinematics
        joints = jointangle(pi/6, -pi/3, pi/6, pi/3)
        end_effector_position = self.forward_kinematics(joints)
        print(end_effector_position)

        # test case for inverse kinematics
        end_effector = position(0.0194, 0.0043, 0.2230)
        joints_angle = self.inverse_kinematics(end_effector)
        print(joints_angle)

        self.move(joints)
        while not rospy.is_shutdown():
            self.rate.sleep()


    def forward_kinematics(self, joints):
        # input: joint angles in (theta1, theta2, theta3, theta4)
        # output: the position of end effector in (x, y, z)
        # hints: access to the angle of the first joint by joints.theta1 
        # add your code here to complete forward kinematics

        return position(x, y, z)


    def inverse_kinematics(self, end_effector):
        # input: the position of end effector in (x, y, z)
        # output: joint angles in (theta1, theta2, theta3, theta4)
        # hints: access to the x position of end effector by end_effector.x
        # add your code here to complete inverse kinematics
        
        return jointangle(theta1, theta2, theta3, pi/3)


    def move(self, joints):
        rospy.loginfo("Manipulator demo")
        for i in range(50):
            if i >= 10:
                self.pub_joint1.publish(joints.theta1)
            if i >= 20:
                self.pub_joint2.publish(joints.theta2)
            if i >= 30:
                self.pub_joint3.publish(joints.theta3)
            if i >= 40:
                self.pub_joint4.publish(joints.theta4)
            self.rate.sleep()


    def reset(self):
        rospy.loginfo("Return to the origin")
        for i in range(50):
            if i >= 10:
                self.pub_joint4.publish(0)
            if i >= 20:
                self.pub_joint3.publish(0)
            if i >= 30:
                self.pub_joint2.publish(0)
            if i >= 40:
                self.pub_joint1.publish(0)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        whatever = OpenManipulator()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")