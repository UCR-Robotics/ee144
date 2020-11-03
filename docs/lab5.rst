Lab 5: Inverse Kinematics
=========================

Overview
--------

In this lab, we will continue working on the manipulator and solve the Inverse Kinematics problem. 

Specifically, the task is to solve for a set of feasible joint angles,
given the position of the end effector. 
You can use either the analytical approach (by trigonometry) or the numerical approach (by Newton's method). 
We will provide a couple of test cases for you on autograder. 
The script you submitted should be able to pass all test cases.

Preview: We will shift back to mobile robots and work on motion planning next time.


Submission
----------

#. Submission: group submission via Gradescope

#. Demo: not required

#. Due time: 11:59pm, Nov 16, Monday

#. Files to submit:

   - lab5_report.pdf
   - inverse_kinematics.py

#. Grading rubric:

   + \+ 50%  Clearly describe your approach and explain your code in the lab report.
   + \+ 50%  Implement inverse kinematics and pass test cases.
   + \- 15%  Penalty applies for each late day. 


Autograder
----------

All code submissions will be graded automatically by an autograder uploaded to Gradescope.
Your scripts will be tested on a Ubuntu cloud server using a similar ROS environment.
The grading results will be available in a couple of minutes after submission.

Testing parameters are as follows. 

#. The tolerance for distance error is set to 0.002m (Manhattan distance on x, y, z axes).

   - The autograder will take the maximum of the error in x, y, z axes respectively,
     and check if the maximum error is less than 0.002m. 
   - For example, if the computed position is [0.020, 0.013, 0.298], and the 
     ground truth is [0.021, 0.012, 0.297], it should pass the test.

#. The autograder works in the way that it sends joint angles to the manipulator and 
   reads the results after the manipulator actually moves according to these angles. 
   It will compare the ground truth position with the position after 
   moving according to the computed angles.

#. The time limit is not set in this lab, as the script should be able to get it done in seconds.


Programming Tips
----------------

#. We provide two scripts ``inverse_kinematics.py`` and ``test_inverse_kinematics.py`` for you,
   but only the first one is what you need to complete and submit. 
   The second one is for your testing.

#. To simplify computation, we will regard ``joint4`` as the end effector. 
   In other words, you are given the position of ``joint4`` instead of the actual end effector.
   The specification of the manipulator is attached in the end of the webpage, 
   where you can find which one is ``joint4``.

#. Options for Inverse Kinematics. Please see lecture slides for more information.

   - Analytical approach by trigonometry
   - Numerical approach by Newton's method

#. Two math tools that might be helpful for the analytical approach. 

   - Two-argument arctangent :math:`\theta = arctan(y, x) \in (-\pi, \pi]` (we use ``atan2`` in programming)
   - Law of cosines :math:`\gamma = arccos(\frac{a^2 + b^2 - c^2}{2ab}) \in (0, \pi)`

#. Math library and functions that might be helpful for the numerical approach.

   - ``import sympy as sym`` the math library ``sympy`` can help with symbolic mathematics
   - ``sym.cos`` and ``sym.sin`` the symbolic version operations (distinct from those in ``math`` library)
   - ``sym.symbols`` create symbolic variables
   - ``sym.diff`` compute the derivative of a function with respect to a variable
   - ``sym.lambdify`` transform a symbolic function in sympy into a native Python function 
     (using lambda expression), such that the Python function can be evaluated at given points later on.
   - If you do want to proceed with the numerical approach, you need to read the documentation of these functions
     in order to properly use them in programming. 


Sample Code
-----------

- Open a new terminal and go to your ``ee144f20`` package. 
  We will start from a new python script.

  .. code-block:: bash

    roscd ee144f20/scripts
    touch inverse_kinematics.py
    gedit inverse_kinematics.py

- Please copy and paste the following code, 
  and complete the ``inverse_kinematics`` function in this file.

  .. literalinclude:: ../scripts/inverse_kinematics.py
    :language: python

- We provide another script for testing.  

  .. code-block:: bash

    roscd ee144f20/scripts
    touch test_inverse_kinematics.py
    gedit test_inverse_kinematics.py

- Please copy and paste the following code.
  You can change the ``test_case`` variable to other values for testing.

  .. literalinclude:: ../scripts/test_inverse_kinematics.py
    :language: python


ReactorX 150 Manipulator
------------------------

- Get familiar with the robot model by launching it in Rviz and playing with the joint state publisher. 

  .. code-block:: bash

    roslaunch interbotix_descriptions description.launch robot_name:=rx150 jnt_pub_gui:=true

- To test the script, launch it without the joint state publisher and run the script in another terminal. 

  .. code-block:: bash

    roslaunch interbotix_descriptions description.launch robot_name:=rx150

  .. code-block:: bash

    roscd ee144f20/scripts
    python test_inverse_kinematics.py

- You need to check if the joint angles computed by your script can lead the manipulator move to 
  the required position. 
  To check the end effect position, run the following command in another terminal.

  .. code-block:: bash

    rosrun tf tf_echo /rx150/base_link /rx150/wrist_link


Specification
-------------

The dimension of the ReactorX 150 manipulator is the following.
We take ``joint4`` as the end effector point (instead of the actual gripper). 

.. image:: pics/rx150.png

Two more annotated figures to help you understand the trigonometry. 
The :math:`\theta_1`, :math:`\theta_2` and :math:`\theta_3` marked in the figures
are the joint angles you need to compute.

.. image:: pics/inv_kin1.png

.. image:: pics/inv_kin2.png

