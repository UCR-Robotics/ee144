Lab 6: Waypoint Navigation
==========================

Overview
--------

In this lab, we will work on the first stage of motion planning, 
which is waypoint navigation. 
Recall that in previous lab, you were able to make your robot
move in a rectangle. 
However, most of your code are specifically designed for this rectangular motion,
and cannot be generalized to any sequence of points on the ground.
In this lab, you are asked to write a function that can drive the robot to any point on the ground,
and visit all the given points. 
Also, you need to apply velocity and acceleration constraints to avoid sharp turn and stop,
and try to make the trajectory as smooth as possible.
The requirements described above will be used in the following labs and your final capstone.

Preview: As a comparison, in the next lab, you will need to implement A* algorithm 
to find these points, which is the second stage of motion planning.

Submission
----------

#. Submission: group submission (from one of you) via iLearn, 
   due by the end of the day of next lab

#. Demo: required, due by the end of next lab

#. Files to submit: **(please do not zip, just upload all files)**

   - lab6_report_01.pdf (replace 01 with your team number, **use number 01-18**)
   - waypoint_navigation.py
  
#. Grading rubric:

   - \+ 50%  Clearly describe your approach and explain your code in lab report.
   - \+ 20%  Roughly visit all given waypoints.
   - \+ 10%  Accelerate at the beginning (first waypoint) and decelerate in the end (last waypoint). 
   - \+ 10%  Meet maximum velocity < 0.5m/s and maximum acceleration < 1.0m/(s^2).
   - \+ 10%  The overall trajectory is smooth (no sharp turn or stop, velocity is about constant).
   - \+ 10%  Bonus points will be given if you can maintain constant velocity or have
     polynomial time scaling for each trajectory segment.
   - \- 15%  Penalty applies for each late day. 


Starter Code
------------

- Open a new terminal and go to your ``ee144f19`` package. 
  We will start from a new python script.

  .. code-block:: bash

    roscd ee144f19/scripts
    touch waypoint_navigation.py
    gedit waypoint_navigation.py

- Please copy and paste the following code, then save and close it.

  .. literalinclude:: ../scripts/waypoint_navigation.py
    :language: python

- You need to complete ``move_to_point`` function in this code 
  and run it on your robot. 
  If your code in function ``move_to_point`` works well,
  this script can help your robot visit all given waypoints.
  However, in order to get smooth trajectory, you may want to make some 
  changes to ``move_to_point`` and ``run`` functions. 
  
- For example, add one more argument in ``move_to_point`` function
  to read two points at the same time, which can help decide in advance
  if you want to stop or keep going when reach next point.
  Also, with two upcoming points, 
  you can know if next path segment is a straight line or 1/4 circle,
  and then plan smooth trajectories accordingly. 

.. note::

  In this lab, since we care more about smooth trajectory,
  you do not need to visit exactly the points on the corner when turning.
  If you do so, you need to stop or decelerate at the corner, 
  which is not encouraged in this lab.


Code Snippets
-------------

- In this section, I will introduce a couple useful code snippets 
  that you may want to integrate into your current code.

- If using while loop in your code, please always remember to add ``not rospy.is_shutdown()``.
  This stop condition can prevent you from getting stuck in dead loop. 

  .. code-block:: python

    while not rospy.is_shutdown():
        pass

- Some of you did something special (use some quick and dirty way) 
  for the third edge of square/rectangle,
  in order to handle the 2pi gap of orientation feedback. 
  However, this is not a proper way because it cannot be generalized.
  The following code can help you address this problem, 
  where error is the update error in your PID controller.

  .. code-block:: python

    self.error = self.set_point - current_value
    if self.error > pi:  # designed for orientation feedback (-pi, pi)
        self.error = self.error - 2*pi
    elif self.error < -pi:
        self.error = self.error + 2*pi
    self.P_value = self.Kp * self.error
    self.D_value = self.Kd * ( self.error - self.Derivator)
    self.Derivator = self.error
    PID = self.P_value + self.D_value

- ``try-except`` structure can help you plot your graph 
  when you terminate the script in the middle. 
  This is useful for debugging.

  .. code-block:: python

    try:
        self.run()
    except rospy.ROSInterruptException as error:
        print(error)
    finally:
        self.visualization()

- If you want to reset odometry, you can get it done by just 
  one command line of code (instead of running a script). 
  Please wisely use Tab key to avoid typing error.

  .. code-block:: bash

    rostopic pub /mobile_base/commands/reset_odometry std_msgs/Empty "{}"

  You may want to check the status of your odometry by command

  .. code-block:: bash

    rostopic echo /odom 


Polynomial Time Scling
----------------------

- If you would like to work on polynomial time scaling,
  please read the following hints.

- The 3rd order polynomial planning is good enough
  in order to meet the acceleration constraint.

- You can calculate the coefficients of function :math:`x(t)`
  from an inverse matrix. 
  Please see lecture slides for more information
  on the matrix form of these constraints.

- The result of your coefficients should be some expression
  in terms of ``T``, where ``T`` is the total traveling time 
  picked by you for the current segment. If you select ``T`` 
  and plug it in, then the coefficients will be just numbers.

- At each segment, the resulting :math:`x(t)` will give you
  the position you should be at each time step,
  and :math:`\dot{x}(t)` will give you the velocity.
  These are functions in terms of current time (step) ``t``.
  
- You also need to solve the coefficients for y direction case,
  since we work on 2D plane. 
  Then you will have two sets of coefficients for each segment.

- For each segment between waypoints, 
  you need to be consistent with your velocity constraints.
  For example, the velocity of the start point in current segment
  should be equal to the velocity of the end point in previous segment.
  For the first and last point, you may want to set the velocity
  to be 0.

