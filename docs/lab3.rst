Lab 3: Closed Loop Control
==========================

Overview
--------

In this lab, we are going to apply PID controller on TurtleBot, for the same square-motion
as in lab 2. This time you are supposed to use a "closed-loop" control approach. 
(You have learned this in EE132, which is the prerequisite of EE144.)
Please implement both P controller and PD controller (two varieties of PID controller),
compare the performance and document your results in lab report.

An example of feedback control algorithm is the following.

#. Initialization: set the robot’s desired orientation as :math:`\phi^* = 0`.
#. Define :math:`P_0` to be the current position of the robot.
#. Keep moving forward, while maintaining the robot’s orientation as close to :math:`\phi^*` as possible, 
   until we reach a location that is at a distance of your side length to :math:`P_0`.
#. Set :math:`\phi^* = \phi^* + \pi / 2`, then properly adjust :math:`\phi^*` if necessary.
#. Rotate in place, until the robot’s orientation equals :math:`\phi^*`
   (within some small tolerance threshold), then repeat from Step 2.
#. Terminate if go back to the origin.


Submission
----------

- Submission type: individual submission via iLearn

- Demo: not required

- Due time: at the beginning of next lab session

- Files to submit: **(please do not zip, just upload all files)**

  #. lab3_report_firstname.pdf (please use template and attach python code in the end)
  #. closed_loop_square_p_ctrl.py
  #. closed_loop_square_pd_ctrl.py
  #. closed_loop_circle.py  (if applicable)
  
- Grading rubric:

  + \+ 20%  Clearly describe your approach and explain your code in lab report.
  + \+ 20%  Your script can drive the robot move along the square trajectory.
  + \+ 20%  Implement P controller and discuss the results of different values of ``Kp``.
  + \+ 20%  Implement PD controller and discuss the results of different values of ``Kp`` and ``Kd``.
  + \+ 20%  Plot the trajectory of your robot with ``matplotlib`` in Python.
  + \+ 10%  Bonus points will be given if you can write another script ``closed_loop_circle.py`` 
    (with any kind of controller) to drive the robot move along a circle of 3m radius, 
    plot the trajectory and report the results.
  + \- 15%  Penalty applies for each late day. 

Preview: We will work on open manipulators in Gazebo next time, 
to help you better understand forward kinematics and inverse kinematics.


Sample Code
------------

A sample code is given as the starting point for your implementation. 
Please read carefully the provided code, and understand its functionality. 

- Open a new terminal and go to your ``ee144f19`` package. 
  We will start from a new python script.

  .. code:: bash

    roscd ee144f19/scripts
    touch closed_loop_square_p_ctrl.py
    gedit closed_loop_square_p_ctrl.py

- Please copy and paste the following code, then save and close it.

  .. code:: python

    #!/usr/bin/env python

    from math import pi, sqrt, atan2, cos, sin
    import matplotlib.pyplot as plt

    import rospy
    import tf
    from geometry_msgs.msg import Twist, Pose2D
    from nav_msgs.msg import Odometry

    class Turtlebot():
        def __init__(self):
            rospy.init_node("turtlebot_move")
            rospy.loginfo("Press CTRL + C to terminate")

            self.pose = Pose2D()
            self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

            self.vel = Twist()
            self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)

            self.logging_counter = 0
            self.rate = rospy.Rate(10)
            self.run()
            self.visualization()


        def run(self):
            # Add your code here to adjust your movement based on Pose2D feedback
            pass

        
        def visualization(self):
            # Add your code here to plot trajectory
            pass


        def odom_callback(self, msg):
            # Get (x, y, theta) specification from odometry topic
            quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

            self.pose.theta = yaw
            self.pose.x = msg.pose.pose.position.x
            self.pose.y = msg.pose.pose.position.y

            # Logging once every 100 times
            self.logging_counter += 1
            if self.logging_counter == 100:
                self.logging_counter = 0
                rospy.loginfo("odom: x=" + str(self.pose.x) +\
                    ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))
                # Add your code here to save the trajectory


    if __name__ == '__main__':
        try:
            whatever = Turtlebot()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")


- As mentioned in Lab 2, you can run it two ways. 

  .. code:: bash

    python closed_loop_square_p_ctrl.py

  .. code:: bash

    chmod +x closed_loop_square_p_ctrl.py
    ./closed_loop_square_p_ctrl.py

.. note::

  If you don't want to restart Gazebo all the time during testing, 
  you can use ``Ctrl + R`` to set your robot to its initial pose.
  However, please note that odometry will not be reset by this command.
  You need to `manually reset the odometry 
  <https://answers.ros.org/question/203088/reset-turtlebot-odometry-in-a-python-script/>`_ 
  in your script.


