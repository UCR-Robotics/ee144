Lab 4: Forward Kinematics
=========================

Overview
--------

In this lab, we will switch gears and work on the manipulator. 

Specifically, we will write a script to solve the Forward Kinematics problem.
Given joint angles, the task is to compute the position of the end effector.
You can use Power of Exponential, D-H Parameters or other approaches you like. 
We will provide a couple of test cases for you in autograder. 
The script you submitted should be able to pass all test cases.

Preview: We will team up and work on Inverse Kinematics problem next time.


Submission
----------

#. Submission: individual submission via Gradescope

#. Demo: not required

#. Due time: 11:59pm, Oct 31, Saturday (in one week)

#. Files to submit: (please submit exactly the same file)

   - lab4_report.pdf
   - forward_kinematics.py

#. Grading rubric:

   + \+ 50%  Clearly describe your approach and explain your code in the lab report.
   + \+ 50%  Implement forward kinematics and pass test cases.
   + \- 15%  Penalty applies for each late day. 


ReactorX 150 Manipulator
------------------------

- Get familiar with the robot model by launching it in Rviz and playing with the joint state publisher. 

  .. code-block:: bash

    roslaunch interbotix_descriptions description.launch robot_name:=rx150 jnt_pub_gui:=true

- To test the script, launch it without joint state publisher and run the script in another terminal. 

  .. code-block:: bash

    roslaunch interbotix_descriptions description.launch robot_name:=rx150

  .. code-block:: bash

    roscd ee144f20/scripts
    python test_forward_kinematics.py  [see next section]

- To check the actual end effect position, run the following command in another terminal.

  .. code-block:: bash

    rosrun tf tf_echo /rx150/base_link /rx150/wrist_link


Starter Code
------------

- Open a new terminal and go to your ``ee144f20`` package. 
  We will start from a new python script.

  .. code-block:: bash

    roscd ee144f20/scripts
    touch forward_kinematics.py
    gedit forward_kinematics.py

- Please copy and paste the following code. 

  .. literalinclude:: ../scripts/forward_kinematics.py
    :language: python

- Please complete the ``forward_kinematics`` function in this file.

- We also provide a testing script.  

  .. code-block:: bash

    roscd ee144f20/scripts
    touch test_forward_kinematics.py
    gedit test_forward_kinematics.py

- Please copy and paste the following code, then save and close it.

  .. literalinclude:: ../scripts/test_forward_kinematics.py
    :language: python

- You can change the ``test_case`` variable to other values to test your function.


Specification
-------------

The dimension of the ReactorX 150 manipulator is the following.
We will take joint4 as the end effector point (instead of the actual gripper). 

.. image:: pics/rx150.png
