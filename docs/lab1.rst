Lab 1: Run in Gazebo
====================

Overview
--------

In this lab, we are going to learn ROS basics and try to make a TurtleBot robot
move in Gazebo simulation. 
I hope this could be a good start of working on robots.

Please take your time and think about the following questions while working on lab materials.
They are helpful for your study, and may be asked during the demo.

- How do we define a ROS workspace? (by which file(s) we can tell this is a ROS workspace)
  What are the characteristics of it?

- How do we define a ROS package? (by which file(s) we can tell this is a ROS package)
  What are the characteristics of it?

- How can we go back to a certain ROS package from any directory we are currently working on?

- What do you observed in Gazebo simulator? What happens if the robot collides with obstacles?


Submission is not required in this lab. Demo is required to show me 
that you can teleop your TurtleBot in Gazebo.

Preview: Next time, we are going to learn how to use scripts (under ROS) to control the robot.

.. note::

   If you are new to Linux, the following instructions may be a bit hard to understand at this moment.
   No worries. Please just try it for the time being. We will go through in details Linux basics
   and how to write scripts in the next lab.


ROS Basics
----------

From now on, we assume that you have Ubuntu 16.04 and ROS Kinetic installed already.

- Please open a new terminal, and create a new ROS workspace by the following commands.

   .. code:: bash

      mkdir -p ~/catkin_ws/src
      cd ~/catkin_ws
      catkin_make
      echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
      source ~/catkin_ws/devel/setup.bash

- Have a look at ``catkin_ws`` directory and see what happens.

- Then let's create a new ROS package.

   .. code:: bash
      
      cd ~/catkin_ws/src
      catkin_create_pkg ee144f19 std_msgs rospy

- Have a look at your new package ``ee144f19`` and see what happens.

- After creating a new package, we have to build it.
  This is to tell ROS that "Hey, we have a new package here. Please register me into your system."
  Go back to our workspace and build packages.

   .. code:: bash
      
      cd ~/catkin_ws
      catkin_make

- Now the system knows this ROS package, and you can have access to it anywhere. 
  Try navigating to different directories first, and then go back to this ROS package by ``roscd`` command.
  See what happens to your command line window.

   .. code:: bash
      
      cd
      roscd ee144f19

      cd ~/catkin_ws
      roscd ee144f19
      
      cd ~/Documents
      roscd ee144f19

- Congratulations. You have completed basic/core steps in ROS. 
  Take some time to think how the above steps work.


Set up Gazebo
-------------

- First go to your ``ee144f19`` package.

   .. code:: bash
      
      roscd ee144f19

- Make a new folder and create a launch file.

   .. code:: bash
      
      mkdir launch
      cd launch
      touch gazebo.launch
      gedit gazebo.launch

- Please copy and paste the following scripts, then save it.

  .. code:: xml

     <launch>
       <arg name="world_file" default="worlds/empty.world"/>

       <arg name="urdf" default="$(find turtlebot_description)/robots/kobuki_hexagons_astra.urdf.xacro" />
       <param name="robot_description" command="$(find xacro)/xacro --inorder $(arg urdf)" />

       <!-- include two nodes gazebo (server) and gazebo_gui (client) -->
       <include file="$(find gazebo_ros)/launch/empty_world.launch">
         <arg name="world_name" value="$(arg world_file)"/>
       </include>

       <!-- Gazebo model spawner -->
       <node name="spawn_turtlebot_model" pkg="gazebo_ros" type="spawn_model"
             args="$(optenv ROBOT_INITIAL_POSE) -unpause -urdf -param robot_description -model mobile_base"/>

       <!-- Velocity muxer -->
       <node pkg="nodelet" type="nodelet" name="mobile_base_nodelet_manager" args="manager"/>
       <node pkg="nodelet" type="nodelet" name="cmd_vel_mux"
             args="load yocs_cmd_vel_mux/CmdVelMuxNodelet mobile_base_nodelet_manager">
         <param name="yaml_cfg_file" value="$(find turtlebot_bringup)/param/mux.yaml"/>
         <remap from="cmd_vel_mux/output" to="mobile_base/commands/velocity"/>
       </node>

     </launch>


Run Turtlebot in Gazebo
-----------------------

- First, let's upgrade existing packages and install some dependencies for Turtlebot. 

   .. code:: bash
      
      sudo apt-get update
      sudo apt-get upgrade
      sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator
      sudo apt-get install ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs

- Launch Gazebo simulator and spawn a new robot.
  It may take a while at the first time you open Gazebo, 
  since it needs some time to download the models and maps.

   .. code:: bash
      
      roslaunch ee144f19 gazebo.launch

- Open a new terminal, launch our teleop node.

   .. code:: bash
      
      roslaunch turtlebot_teleop keyboard_teleop.launch

- Nice. You should be able to control the robot now. Spend some time and play with it!

- You can also put some obstacles (objects) in Gazebo simulated environment,
  and make the robot collide with obstacles. See what happens :)


.. note::

   If you are experiencing graphic issues with Gazebo, please try the following command.
   Then close all terminals and try again.

   .. code:: bash
      
      echo "export SVGA_VGPU10=0" >> ~/.profile




