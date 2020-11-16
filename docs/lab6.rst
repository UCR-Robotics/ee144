Lab 6: Trajectory Generation
============================

Overview
--------

In this lab, we will focus on how to generate **smooth trajectories** using polynomial time scaling. 

Specifically, the task is to implement the 3rd order polynomial time scaling and apply it for each segment of the trajectory
and for both x and y coordinates. Waypoints will be provided, and should be included into the boundary constraints
when generating the trajectries.

Preview: Next time we will learn how to use A* algorithm to search for the waypoints, when a map is given.


Submission
----------

#. Submission: group submission via Gradescope

#. Demo: not required

#. Due time: 11:59pm, Nov 23, Monday

#. Files to submit:

   - lab6_report.pdf (please include the plot of trajectory)
   - trajectory_generation.py

#. Grading rubric:

   + \+ 30%  Clearly describe your approach and explain your code in the lab report.
   + \+ 20%  Plot the trajectory and discuss the results under different parameters.
   + \+ 50%  Pass all waypoints using 3rd order polynomial trajectories.
   + \- 15%  Penalty applies for each late day. 


Sample Code
-----------

- Open a new terminal and go to your ``ee144f20`` package. 
  We will start from a new python script.

  .. code-block:: bash

    roscd ee144f20/scripts
    touch trajectory_generation.py
    gedit trajectory_generation.py

- Please copy and paste the following code.

  .. literalinclude:: ../scripts/trajectory_generation.py
    :language: python

- This script is following the same structure as the one used in Lab 3, 
  except for the changes under ``run`` function.

- You need to complete the ``move_to_point`` function in this code,
  and make sure the robot can pass all the waypoints. 
  The ``polynomial_time_scaling_3rd_order``
  function is provided for your information only. You may or may not use it.


Polynomial Time Scling
----------------------

- You can calculate the coefficients of function :math:`x(t)`
  from an inverse matrix. 
  Please see lecture slides for more information
  about the matrix form of the constraints.

- The result of the coefficients should be
  in terms of ``T``, where ``T`` is the desired traveling time 
  of the current segment of trajectory.

- At each segment, the resulting :math:`x(t)` will give you
  the position you should be at each time step,
  and :math:`\dot{x}(t)` will give you the velocity.
  These are functions in terms of current time ``t``.
  
- Coefficients need to be solved for x and y respectively, 
  for each trajectory segment with time scaling [0, T].

