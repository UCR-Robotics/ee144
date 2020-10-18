Lab 3: Closed Loop Control
==========================

Overview
--------

In this lab, we are going to apply closed-loop control to trajectory tracking. 
A ROS Subscriber is included in the script to receive real-time odometry feedback from Gazebo simulator.
With the odom/pose updates, the robot can adjust its moving direction accordingly and check if 
the desired waypoint has been reached.

Specifically, the task is to implement a PD controller (a variant of PID controller that removes "I" component)
to track the square shape trajectory. 
Same as in Lab 2, the waypoints are [4, 0], [4, 4], [0, 4], [0, 0], and the sequence does matter.
After task completion, the robot should stop at the origin and the Python script should exit gracefully. 
Please plot the trajectory (using another provided Python script) and discuss your results in the lab report. 

Bonus points will be given for the completion of an additional task of tracking a circular trajectory.
The circular trajectory should be centered at point [0, 2] and have a radius of 2m; counter-clockwise sequence. 
(Hint: You will need to write a function to generate continuous waypoints to track.)

Preview: We will play with manipulators in Gazebo next time. 
(Specifically, to solve a Forward Kinematics problem.)


Submission
----------

#. Submission: individual submission via Gradescope

#. Demo: not required

#. Due time: 11:59pm, Oct 24, Saturday (in one week)

#. Files to submit: (please use exactly the same filename; case sensitive)

   - lab3_report.pdf
   - closed_loop.py
   - closed_loop_circle.py (optional; bonus points)

#. Grading rubric:

   + \+ 30%  Clearly describe your approach and explain your code in the lab report.
   + \+ 20%  Plot the trajectory and discuss the results of different values of ``Kp`` and ``Kd``.
   + \+ 40%  Implement PD controller; visit four vertices of the square trajectory with error < 0.1m. 
     Partial credits will be given according to the number of vertices visited.
   + \+ 10%  The script can complete the task on time and exit gracefully.
   + \+ 10%  Bonus points for tracking a circular trajectory;
     plot the trajectory and report the results.
   + \- 15%  Penalty applies for each late day. 


Autograder
----------

All code submissions will be graded automatically by an autograder uploaded to Gradescope.
Your scripts will be tested on a Ubuntu cloud server using a similar ROS + Gazebo environment.
The grading results will be available in a couple of minutes after submission.

Testing parameters are as follows. 

#. The tolerance for distance error is set to 0.1m (considering that this is closed-loop control).

   - For example, passing point [3.96, 3.94] is approximately equivalent to passing point [4, 4].

#. The required waypoints for square trajectory are [4, 0], [4, 4], [0, 4] and [0, 0]; sequence matters.

#. The circular trajectory should be centered at point [0, 2] and have a radius of 2m. 
   10 waypoints are evenly selected on this trajectory, in a counter-clockwise sequence.

#. The time limit for the submitted script is set to 5 mins.

   - If running properly, the task in this lab can be done in about 2 mins, based on my testing.
   - If running timeout, the script will be terminated and a 10% penalty will apply.

.. note::

  It is required to use closed-loop control (i.e. PD controller) to track the trajectory. 
  Finely tuned open-loop control script may also pass the autograder testing, but is not allowed.
  All scripts will be double checked when grading manually. 
  Penalty will apply if such script is found.


PID Control
-----------


An example of feedback control algorithm is the following.

#. Initialization: set the robot’s desired orientation as :math:`\phi^* = 0`.
#. Define :math:`P_0` to be the current position of the robot.
#. Keep moving forward, while maintaining the robot’s orientation as close to :math:`\phi^*` as possible, 
   until we reach a location that is at a distance of your side length to :math:`P_0`.
#. Set :math:`\phi^* = \phi^* + \pi / 2`, then properly adjust :math:`\phi^*` if necessary.
#. Rotate in place, until the robot’s orientation equals :math:`\phi^*`
   (within some small tolerance threshold), then repeat from Step 2.
#. Terminate if go back to the origin.

A discrete PD controller implementation in Python is provided (partially). 
To make it work, you need to understand the PD control algorithm 
and fill in the ``update`` function.
You may or may not use it in your own implementation. 

  .. literalinclude:: ../scripts/controller.py
    :language: python


Sample Code
------------

A sample code is provided as the starting point for your implementation. 
Please read carefully the provided code, and understand its functionality. 

- Open a new terminal and go to your ``ee144f20`` package. 
  We will start from a new python script.

  .. code-block:: bash

    roscd ee144f20/scripts
    touch closed_loop.py
    gedit closed_loop.py

- Please copy and paste the following code, then save and close it.

  .. literalinclude:: ../scripts/closed_loop.py
    :language: python

- Please make changes to the ``run`` function to complete the task in this lab.
  Once finished, you can run it two ways as introduced in Lab 2.
  (Remember to bring up the robot before running the script.)

  .. code-block:: bash

    python closed_loop.py

  .. code-block:: bash

    chmod +x closed_loop.py
    ./closed_loop.py


Sample Code Explained
---------------------

- Odometry works in the way that it counts how many rounds the wheel rotates. 
  Therefore, if you lift and place a robot from one place to another, it will still "think" 
  that it was at the original place.

- In Gazebo simulator, using command ``Ctrl + R`` to reset the robot is similar to 
  lifting the robot and placing it to the origin, where the odometry will still report 
  the previous history record. Therefore, to obtain a correct odometry feedback for the 
  new run, we need to reset the odometry to zero. 

  .. code-block:: python

    self.reset_pub = rospy.Publisher('mobile_base/commands/reset_odometry', Empty)
    for i in range(10):
        self.reset_pub.publish(Empty())
        self.rate.sleep()

- In Lab 2, we have learned how to use ROS Publisher to send a message out. 
  ROS Subscriber is the one on the other side to receive and process the messages.
  The required arguments are the topic name ``odom``, 
  the message type ``Odometry``, and a pointer to the callback function.
  The callback function will be executed (asynchronously; in another thread) whenever a new message is received.
  We leverage the shared variables in Turtlebot class to store the latest pose of the robot.
  Note that the ``msg`` argument in the callback function is of the type ``Odometry``. 
  The definition is specified in 
  `the ROS Wiki documentation <http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html>`_.

  .. code-block:: python

    self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        pass

- In the ``try-except`` structure, ``finally`` is the keyword to indicate that the following
  code block will be executed regardless if the try block raises an error or not. 
  In this case, we want to save the trajectory even when the robot stops at halfway.

  .. code-block:: python

    finally:
        # save trajectory into csv file
        np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')


Visualization
-------------

We provide a separate Python script to help visualize the trajectory from the saved csv file. 
Please do not submit this file and do not include it in the script you plan to submit,
as it will block the autograder until running timeout.

.. literalinclude:: ../scripts/visualization.py
  :language: python


