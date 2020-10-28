Lab 4: Forward Kinematics
=========================

Overview
--------

In this lab, we will switch gears and work on the manipulator. 

Specifically, we will write a script to solve the Forward Kinematics problem.
Given joint angles and the dimension of the manipulator, 
the task is to compute the position of the end effector.
You can use Product of Exponentials, D-H Parameters or other approaches you like. 
We will provide a couple of test cases for you on autograder. 
The script you submitted should be able to pass all test cases.

Preview: We will team up and work on Inverse Kinematics next time.


Submission
----------

#. Submission: individual submission via Gradescope

#. Demo: not required

#. Due time: 11:59pm, Nov 2, Monday

#. Files to submit: (please submit exactly the same file)

   - lab4_report.pdf
   - forward_kinematics.py

#. Grading rubric:

   + \+ 50%  Clearly describe your approach and explain your code in the lab report.
   + \+ 50%  Implement forward kinematics and pass test cases.
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

#. Three test cases are visible to you; two test cases are hidden. 
   The hidden ones are designed to test some "corner cases" and 
   to make sure there is no hard-coded computation in the submitted script.

#. The time limit is not set in this lab, as the script should be able to get it done in seconds.


Programming Tips
----------------

#. We provide two scripts ``forward_kinematics.py`` and ``test_forward_kinematics.py`` for you,
   but only the first one is what you need to complete and submit. 
   The second one is for your testing.

#. To simplify computation, we will regard ``joint4`` as the end effector. 
   In other words, you need to return the position of ``joint4`` instead of the actual end effector.
   The specification of the manipulator is attached in the end of the webpage, 
   where you can find which one is ``joint4``.

#. Options for Forward Kinematics

   - Directly write down the transformation matrices between joints and links.
   - Product of Exponentials in space frame.
   - Product of Exponentials in body frame.
   - Formulate Denavit-Hartenberg parameters between each frame and multiply all transformation matrices.
   - You can either compute everything using program, or pre-compute some matrices by hand.

#. Please pay attention to the data type used in your computation.

   - If two matrices ``A`` and ``B`` are of the type ``np.array``, 
     then ``A * B`` will **not** perform matrix multiplication, but element-wise multiplication.

   - If two matrices ``A`` and ``B`` are of the type ``np.array``, 
     then ``np.dot(A, B)`` **will** perform matrix multiplication.

   - If two matrices ``A`` and ``B`` are of the type ``np.matrix``, 
     then ``A * B`` **will** perform matrix multiplication.

#. Functions available in ``numpy`` and ``modern_robotics`` library.

   - ``np.cross(A, B)`` the cross product of two vectors
   - ``np.concatenate([A, B])`` the concatenation of two vectors
   - ``mr.VecTose3(S)`` converts a 6x1 twist vector S to a 4x4 matrix in se(3) 
   - ``mr.MatrixExp6(M)`` the matrix exponential of a 4x4 matrix in se(3)


ReactorX 150 Manipulator
------------------------

- Get familiar with the robot model by launching it in Rviz and playing with the joint state publisher. 
  (You should have all software packages installed in Lab 1.)

  .. code-block:: bash

    roslaunch interbotix_descriptions description.launch robot_name:=rx150 jnt_pub_gui:=true

- To test the script, launch it without the joint state publisher and run the script in another terminal. 

  .. code-block:: bash

    roslaunch interbotix_descriptions description.launch robot_name:=rx150

  .. code-block:: bash

    roscd ee144f20/scripts
    python test_forward_kinematics.py  [see next section]

- To check the actual end effect position, run the following command in another terminal.

  .. code-block:: bash

    rosrun tf tf_echo /rx150/base_link /rx150/wrist_link


Sample Code
-----------

- Open a new terminal and go to your ``ee144f20`` package. 
  We will start from a new python script.

  .. code-block:: bash

    roscd ee144f20/scripts
    touch forward_kinematics.py
    gedit forward_kinematics.py

- Please copy and paste the following code, 
  and complete the ``forward_kinematics`` function in this file.

  .. literalinclude:: ../scripts/forward_kinematics.py
    :language: python

- We provide another script for testing.  

  .. code-block:: bash

    roscd ee144f20/scripts
    touch test_forward_kinematics.py
    gedit test_forward_kinematics.py

- Please copy and paste the following code.
  You can change the ``test_case`` variable to other values for testing.

  .. literalinclude:: ../scripts/test_forward_kinematics.py
    :language: python


Specification
-------------

The dimension of the ReactorX 150 manipulator is the following.
We will take ``joint4`` as the end effector point (instead of the actual gripper). 

.. image:: pics/rx150.png
