Lab 1: Run in Gazebo
====================

Overview
--------

In this lab, we are going to create a ROS workspace and a ROS package, 
launch a Turtlebot robot and play with it, launch a robot arm and play with it. 

While following the step-by-step tutorials, please take your time to think about 
what you are doing and what happens in each step, with the help of Google if necessary.

Preview: Next week we are going to learn how to write Python scripts to control the Turtlebot robot.


Submission
----------

#. Submission: individual submission via Gradescope

#. Demo: required (one for Turtlebot and one for robot arm)

#. Due time: 5:00pm, Oct 7, Friday

#. Files to submit: (please use exactly the same filename; case sensitive)

   - lab1_report.pdf (please use the provided Word template and export to pdf)

#. Grading rubric:

   - \+ 20%  Have the ROS workspace and the ROS package ready.
   - \+ 30%  Launch a Turtlebot robot in Gazebo and demo to TAs how you play with it.
   - \+ 30%  Launch a ReactorX 150 robot arm in Gazebo and demo to TAs how you play with it.
   - \+ 20%  Write down what you have learned, your findings and thoughts in lab report.
   - \- 15%  Penalty applies for each late day (up to two days). 


ROS Basics
----------

From now on, we assume that you have already installed Ubuntu 16.04 and ROS Kinetic.

- Please open a new terminal, and create a new ROS workspace by the following commands (run them line by line).

  .. code-block:: bash

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/catkin_ws/devel/setup.bash

- Take a look at ``catkin_ws`` directory and see what happens. 
  You can use ``ls`` command to see the files in this directory, or use ``ls -a`` to see all files including hidden files.
  Alternatively, you can open File Explorer and navigate to this folder, and use ``Ctrl + H`` to see hidden files.
  The file ``.catkin_workspace`` was created to tell the system that this current directory is a ROS workspace.

.. note::

  If you are fresh new to Linux, the instructions might be a bit hard to understand at this moment.
  No worries. Please just try it for the time being and you will have a better understanding as we move on.
  You can always ask me any question you want during lab sessions to help you better understand lab materials. 
  (Remember that there is no stupid question; even for those "too simple to ask" questions, We are always happy to answer.)

- Next let's create a new ROS package.

  .. code-block:: bash
      
    cd ~/catkin_ws/src
    catkin_create_pkg ee144f22 std_msgs rospy

- Take a look at your new package ``ee144f22`` and see what happens. You should be able to see a ``package.xml`` file
  and a ``CMakeLists.txt`` file. Open them and take a quick look. 
  You may use Google to help you build up a high-level understanding.

- After creating a new package, we can go back to our workspace and build this package.
  This is to tell ROS that "Hey, we have a new package here. Please register it into the system."

  .. code-block:: bash
      
    cd ~/catkin_ws
    catkin_make

- Now the system knows this ROS package, so that you can have access to it anywhere. 
  Try navigating to different directories first, and then go back to this ROS package by ``roscd`` command.
  See what happens when running the following commands.

  .. code-block:: bash
      
    cd
    roscd ee144f22

    cd ~/catkin_ws
    roscd ee144f22
      
    cd ~/Documents
    roscd ee144f22

- Congratulations. You have completed the basic ROS tutorials.
  Take some time to think about how the above steps work.


Set up Turtlebot in Gazebo
--------------------------

- First let's upgrade existing packages and install some dependencies for Turtlebot. 

  .. code-block:: bash
      
    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator
    sudo apt-get install ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs

- Navigate to your ``ee144f21`` package and create a new folder and a new launch file.

  .. code-block:: bash
      
    roscd ee144f22
    mkdir launch
    cd launch
    touch gazebo.launch
    gedit gazebo.launch

- Please copy and paste the following script, then save it.

  .. literalinclude:: ../launch/gazebo.launch
    :language: xml


Run Turtlebot in Gazebo
-----------------------

- Launch Gazebo simulator and spawn a new robot by the following command.
  It may take a while at the first time you open Gazebo, 
  since it will need to download some models and world environments.

  .. code-block:: bash
      
    roslaunch ee144f22 gazebo.launch

.. note::

  If you experienced graphic issues in Gazebo, please run the following command for once.
  Then close all terminals and try again.

  .. code-block:: bash
      
    echo "export SVGA_VGPU10=0" >> ~/.bashrc

  If the issue persists, please shutdown your VM, go to VM settings and allocate more resources
  (Processor Cores, Memory, Graphics Memory). If the issue still persists, please disable 
  "3D Acceleration" in Display settings.
  
- Once the robot is successfully spawned in Gazebo, we can open a new terminal and launch the teleop node.

  .. code-block:: bash
      
    roslaunch turtlebot_teleop keyboard_teleop.launch

- Keep the teleop terminal open (selected) and you should be able to control the robot using keyboard now. 
  The teleop program in this terminal takes in whatever keys you entered and 
  converts them into velocity commands to send to the robot. Now spend some time playing with it! 
  (Don't send the keyboard teleop commands to the Gazebo window, it won't work; send commands to the terminal)

- You can also put some obstacles (objects) in Gazebo simulation environment,
  and drive the robot to collide with obstacles. See what happens :)

.. note::

  To terminate the programs running in the terminal, please use ``Ctrl + C`` and wait for a moment
  (it does take about 10s for Gazebo to shut down). 
  If the terminal is closed without terminating the programs properly 
  (meaning that the programs are still running in the back-end),
  you will see a Gazebo crash error in the next run.


Set up robot arm in Gazebo
--------------------------

- First let's download the ROS packages for the robot arm.  

  .. code-block:: bash
      
    cd ~/catkin_ws/src
    git clone https://github.com/UCR-Robotics/interbotix_ros_arms.git

- We can install the dependencies by the following commands.

  .. code-block:: bash
      
    cd ~/catkin_ws
    rosdep update --include-eol-distros
    rosdep install --from-paths src --ignore-src -r -y

- We need to add one more package that is not currently supported by ``rosdep`` install.
  (BTW, this *modern_robotics* library is developed by the authors of our textbook *Modern Robotics*.
  It contains the Python implementation of some common operations. We will learn them in lectures as well.)

  .. code-block:: bash
      
    sudo apt install python-pip
    sudo pip install modern_robotics

- Lastly, with all dependencies ready, we can build the ROS package by the following commands.

  .. code-block:: bash
      
    cd ~/catkin_ws
    catkin_make


Play with robot arm in Gazebo
-----------------------------

- Launch the ReactorX 150 robot arm in Gazebo by the following command.

  .. code-block:: bash
      
    roslaunch interbotix_moveit interbotix_moveit.launch robot_name:=rx150 use_gazebo:=true

- You will see the robot arm is ready in Gazebo but the RViz (the visualization software used in ROS) is still pending.
  This is because it is still waiting for Gazebo to start simulation. 
  In the bottom left of Gazebo window, you will see a small **Play ▶ button**. Click it to let it run!

- Once Gazebo starts simulation, the RViz will prompt you two panels on the left and a visualization of the robot arm on the right. 
  On the top left panel, go to "MotionPlanning" -> "Planning Request" -> "Query Goal State" and check this box. 
  Then you can drag the "ball" on the tip of the robot arm to wherever you want it to go. 

- Once a goal pose is set, in the bottom left panel, go to "Planning" tab and try buttons "Plan", "Execute", or "Plan and Execute". 
  Cool! The software can figure out a path for the arm to follow and reach the exact goal pose you just set.
  Spend some time playing with it!

- You can also take a look at Gazebo to see the current status of the robot arm. 
  RViz provides a tool for better interaction, but only Gazebo shows the real physical status.

- Have fun!!

