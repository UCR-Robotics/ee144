Lab 1: Run in Gazebo
====================

Overview
--------

In this lab, we are going to learn ROS basics and try to make a TurtleBot robot
move in Gazebo simulation. I hope this could be a good start of working on robots.

Please take your time and think about the following questions while working on lab materials.
They are helpful for your study, and will be asked during the demo.

- How do we define a ROS workspace? (by which file(s) we can tell this is a ROS workspace)
  What are the characteristics of it?

- How do we define a ROS package? (by which file(s) we can tell this is a ROS package)
  What are the characteristics of it?

- How can we go back to a certain ROS package from any directory we are currently working on?

- What do you observed in Gazebo simulator? What happens if the robot collides with obstacles?

Submission
----------

- Submission: not required

- Demo: required (to show me that you can teleop your TurtleBot in Gazebo)

- Due time: at the end of next lab session
  
- Grading rubric:

  + \+ 50%  Have ROS environment and TurtleBot in Gazebo ready.
  + \+ 40%  You can teleop your TurtleBot in Gazebo and make it move.
  + \+ 10%  Answer the above questions during demo.
  + \- 15%  Penalty applies for each late day. 

Preview: Next time, we are going to learn how to use scripts (under ROS) to control the robot.

.. note::

  If you are new to Linux, the following instructions may be a bit hard to understand at this moment.
  No worries. Please just try it for the time being. We will go through in details Linux basics
  and how to write scripts in the next lab.


ROS Basics
----------

From now on, we assume that you have Ubuntu 16.04 and ROS Kinetic installed already.

- Please open a new terminal, and create a new ROS workspace by the following commands.

  .. code-block:: bash

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/catkin_ws/devel/setup.bash

- Have a look at ``catkin_ws`` directory and see what happens.

- Then let's create a new ROS package.

  .. code-block:: bash
      
    cd ~/catkin_ws/src
    catkin_create_pkg ee144f20 std_msgs rospy

- Have a look at your new package ``ee144f20`` and see what happens.

- After creating a new package, we have to build it.
  This is to tell ROS that "Hey, we have a new package here. Please register me into your system."
  Go back to our workspace and build packages.

  .. code-block:: bash
      
    cd ~/catkin_ws
    catkin_make

- Now the system knows this ROS package, and you can have access to it anywhere. 
  Try navigating to different directories first, and then go back to this ROS package by ``roscd`` command.
  See what happens to your command line window.

  .. code-block:: bash
      
    cd
    roscd ee144f20

    cd ~/catkin_ws
    roscd ee144f20
      
    cd ~/Documents
    roscd ee144f20

- Congratulations. You have completed basic/core steps in ROS. 
  Take some time to think how the above steps work.


Set up Gazebo
-------------

- First go to your ``ee144f20`` package.

  .. code-block:: bash
      
    roscd ee144f20

- Make a new folder and create a launch file.

  .. code-block:: bash
      
    mkdir launch
    cd launch
    touch gazebo.launch
    gedit gazebo.launch

- Please copy and paste the following scripts, then save it.

  .. literalinclude:: ../launch/gazebo.launch
    :language: xml


Run Turtlebot in Gazebo
-----------------------

- First, let's upgrade existing packages and install some dependencies for Turtlebot. 

  .. code-block:: bash
      
    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator
    sudo apt-get install ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs

- Launch Gazebo simulator and spawn a new robot.
  It may take a while at the first time you open Gazebo, 
  since it needs some time to download the models and maps.

  .. code-block:: bash
      
    roslaunch ee144f20 gazebo.launch

- Open a new terminal, launch our teleop node.

  .. code-block:: bash
      
    roslaunch turtlebot_teleop keyboard_teleop.launch

- Nice. You should be able to control the robot now. Spend some time and play with it!

- You can also put some obstacles (objects) in Gazebo simulated environment,
  and make the robot collide with obstacles. See what happens :)


.. note::

  If you are experiencing graphic issues with Gazebo, please try the following command.
  Then close all terminals and try again.

  .. code-block:: bash
      
    echo "export SVGA_VGPU10=0" >> ~/.profile

  Another reason could be limited hardware resources allocated to this VM. 
  You may shutdown your VM first, then go to the settings of this VM.
  Please allocate as more Memory and Processors as possible.
  You may also try to allocate more Graphics memory (in Display).
  
  



