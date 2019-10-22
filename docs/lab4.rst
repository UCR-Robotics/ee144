Lab 4: Open Manipulator
=======================

Overview
--------

In this lab, we will switch gears and work on manipulators. 
This lab aims to provide you some experience of working on manipulators,
and help you better understand forward kinematics and inverse kinematics.
With the help of the dimension of the manipulator, you need to figure out 
how to calculate the position of the end effector given joint angles (forward kinematics), 
and how to get a solution of joint angles given the position of the end effector (inverse kinematics).
Please refer to your lecture slides for more information about detailed approaches.

Submission
----------

- Submission type: individual submission via iLearn

- Demo: not required

- Due time: at the beginning of next lab session

- Files to submit: **(please do not zip, just upload all files)**

  #. lab4_report_firstname.pdf (please use template and attach python code in the end)
  #. open_manipulator.py
  
- Grading rubric:

  + \+ 50%  Clearly describe your approach and explain your code in lab report.
  + \+ 30%  Implement forward kinematics and pass the test case.
  + \+ 20%  Implement inverse kinematics and pass the test case.
  + \+ 10%  Bonus points will be given if your algorithm can handle special cases (unreachable, multiple solutions).
  + \- 15%  Penalty applies for each late day. 

Preview: We will work on real robots in teams next time.


Setup
-----

According to `the official wiki website 
<http://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/>`_, 
we need to install the following packages and dependencies.

- Install dependencies from apt-get.

  .. code:: bash

    sudo apt-get update
    sudo apt-get install ros-kinetic-ros-controllers ros-kinetic-gazebo-* 
    sudo apt-get install ros-kinetic-moveit* ros-kinetic-industrial-core


- Install ROS packages from official GitHub repositories.

  .. code:: bash

    cd ~/catkin_ws/src/
    git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
    git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
    git clone https://github.com/ROBOTIS-GIT/open_manipulator.git
    git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
    git clone https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
    git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git
    cd ~/catkin_ws && catkin_make

- Load OpenManipulator-X on Gazebo simulator and **click on Play ▶ button**.

  .. code:: bash

    roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch

- Open two new terminals and play with it by keyboard teleoperation.

  .. code:: bash

    roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false

    roslaunch open_manipulator_teleop open_manipulator_teleop_keyboard.launch

- Once you are familiar with this manipulator, close all terminals and go to the following steps.


Sample Code
------------

A sample code is given as the starting point for your implementation. 
Please read carefully the provided code, and understand its functionality. 

- Open a new terminal and go to your ``ee144f19`` package. 
  We will start from a new python script.

  .. code:: bash

    roscd ee144f19/scripts
    touch open_manipulator.py
    gedit open_manipulator.py

- Please copy and paste the following code, then save and close it.

  .. code:: python

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
            end_effector = position(0.018, 0.004, 0.222)
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
            
            return jointangle(theta1, theta2, theta3, theta4)


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


- Please add your code in ``self.forward_kinematics`` and ``self.inverse_kinematics`` functions
  to complete the script.

- As mentioned in Lab 2, you can run it two ways. 

  .. code:: bash

    python open_manipulator.py

  .. code:: bash

    chmod +x open_manipulator.py
    ./open_manipulator.py

- You can see the pose of **link**, and angle of **joint** in Gazebo, 
  by selecting the object and check the property on the left sidebar.

- Have fun!


.. note::

  You need to launch manipulator in Gazebo before running your script. 
  (No need to launch controller.)

  .. code:: bash

    roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch

  Remember to **click on Play ▶ button** to start simulation!



Specification
-------------

The dimension of the open manipulator is the following.

.. image:: pics/manipulator.jpg

