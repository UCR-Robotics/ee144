Lab 5: Hi! Robot Arm!
=========================

Overview
--------

In this lab, we will continue working on the manipulator and play with the physical robot in **WCH125**. 
Specifically, the task is to successfully (1) define a Publisher to test your algorithms in simulation and (2) communicate with the physical robots.

**Please make sure you have a laptop that installed Ubuntu with you per team.**


Submission
----------

#. Submission: group submission via Gradescope

#. Demo: During the lab time.

#. Due time: 23:59 pm, Nov 17, Friday

#. Files to submit:

   - lab5_report.pdf

#. Grading rubric:

   + \+ 25%  Successfully define the Publisher to test your scripts. 
   + \+ 25%  Successfully launch the physical robot and drive it to `home` state.
   + \+ 50%  Write down what you have learned, your findings and thoughts in the lab report.
   + \- 15%  Penalty applies for each late day. 

Access to Lab and Lab Safety
----------------------------

#. Please read the **Lab Safety** requirements in `logistics <logistics.html>`_ carefully.

#. PLease sign in and sign out when you enter/leave the lab room. 
   (There is a sign-in sheet near the door to WCH 125.) 
   The lab rule is enforced by the department.

#. There are 12 ReactorX 150 manipulators in the lab. 
   Each team will be assigned to one robotic arm, given the number ID that had on Lab4.

Write a Publisher and test your code
--------------------------------------------------

- We provide the scripts for testing your algorithms.  

  .. code-block:: bash

    roscd ee144f23/scripts
    touch test_kinematics.py
    gedit test_kinematics.py

- Please copy and paste the following code. Then fill in your code to define a Publisher ``self.joint_pub`` 
  with the topic as ``/rx150/joint_states``, message type as ``JointState`` and queue_size as 10.

  .. literalinclude:: ../scripts/test_kinematics.py
    :language: python

- Now, launch the arm in Rviz to make sure your code woking properly. 

  .. code-block:: bash

    roslaunch interbotix_descriptions description.launch robot_name:=rx150

- Open a new terminal and run the following code, you should see your outputs of the Forward Kinematics problem and the Inverse Kinematics problem;

  .. code-block:: bash

    roscd ee144f23/scripts
    python test_kinematics.py

- Then check and compare with the actual end effect position, run the following command in another terminal.

  .. code-block:: bash

    rosrun tf tf_echo /rx150/base_link /rx150/wrist_link


Hardware Setup
--------------

Most of the setup of the robot arm is ready. You just need to power it and connect it to your laptop.

#. The manipulator is placed on the sturdy and flat table.
   Don't move the arm out of the table and make sure that there are no obstacles within the workspace of it.

#. Plug the 12V power cable into an outlet and insert the barrel plug into the barrel jack on the X-series power hub.
   Then plug the micro-usb cable into the U2D2.
   While don't plug in the other side to your laptop now.
   Both of the two ports are located under the see-through acrylic on the base of the robot.

#. Copy over the udev rules to the right directory so your laptop (or the VMware) could recognize the U2D2.

   .. code-block:: bash

       $ sudo cp ~/catkin_ws/src/interbotix_ros_arms/interbotix_sdk/10-interbotix-udev.rules /etc/udev/rules.d
       $ sudo udevadm control --reload-rules && udevadm trigger

#. Now plug in the micro-usb cable to your computer. You should see the LEDs flash red.
   You could also use ``lsusb`` to see whether the ``Bus 001 Device 002: ID 0403:6014 Future Technology Devices International, Ltd FT232H Single HS USB-UART/FIFO IC`` is listed.


Communicate with the Physical Robot
-----------------------------------

Now you are ready to play with the real robot. 
(You should have all software packages installed in Lab 1.)
Here we use Moveit to help us achieve the desired pose of arm or gripper.

#. Launch the driver node of the actual robot and Moveit

   .. code-block:: bash
    
     roslaunch interbotix_moveit interbotix_moveit.launch robot_name:=rx150 use_actual:=true dof:=5

   You should see the light color changes and all the motors in the robot are torqued on..

#. Now you could play with Moveit on your laptop to plan the trajectory. 
   Don't forget to click ``update`` after you set the ``goal state``.
   PLease set the ``goal state`` as ``home`` for demo.
   **Note: be careful to avoid collisions when you set the goal states and during the planning**.
   **(Run** ``plan`` 
   **first to see how it will perform in Rviz, only when you make sure your planned trajectory wouldn't cause any collisions, click**
   ``Execute``). 

#. Shutdown the previous procedure and we will try one example using the ``moveit_python_interface``. Open a new terminal

	.. code-block:: bash

		$ roslaunch interbotix_moveit_interface moveit_interface.launch robot_name:=rx150 use_python_interface:=true use_actual:=true
    
	Once you launch the file, you will find one line "============ Press Enter to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ...". 
	You might miss it because everything is getting launched at the same time. Just scroll through the text to find it. 
	Then press "Enter", you will see a new line and press "Enter" then you could see your arm moves both in the RViz and the actual robot. 
	The rest can be done in the same manner. 

#. When you are done with your work, put the arm back safely for further use.  
   **Please be very careful as the arm will collapse once you run the command. So you should hold the arm manually before it falls down.**
   Firstly, run ``$ rosservice call /rx150/torque_joints_off`` to torque off the motors,
   Then manually put it back to the safe position as it originally was.
   Finally, shut down (ctrl+C) your launch process and unplug the power cords.



