Lab 8: Perception
=================

Overview
--------

In this lab, we will introduce how to do perception on TurtleBot robot with laser scanner and depth camera.
The robot is equipped with RPLidar A2 and Obbrec Astra Pro camera.

Please take a screenshot for each question below and answer in your lab report.

#. What is the difference (and/or relation) between ``map``, ``odom``, ``base``
   and ``sensors`` coordinate frames? Please give explaination to each of them.
#. What is the data format for laser scanner? What are the pros and cons of laser scanner?
#. What is the data format for depth camera? What are the pros and cons of depth camera?


Preview: We will work on ROS navigation stack next time.


Submission
----------

#. Submission: group submission (from one of you) via iLearn, 
   due by the end of the day of next lab

#. Demo: required, due by the end of next lab

#. Files to submit: **(please do not zip, just upload all files)**

   - lab8_report_01.pdf (replace 01 with your team number, **use number 01-18**)
  
#. Grading rubric:

   - \+ 50%  Clearly describe your approach and explain necessary steps in lab report.
   - \+ 30%  Go through the instructions and show demo to me.
   - \+ 20%  Answer the above conceptual questions in lab report.
   - \- 15%  Penalty applies for each late day. 


Sensors Setup
-------------

- Open a new terminal and remote login to your robot with ``-X`` flag.

  .. code:: bash

    ssh -X ee144-nuc01@10.40.2.21

- We need to create a ROS workspace and install ROS packages.
  **If the workspace or package already exist, please skip the corresponding steps.**
  The following steps should be executed on your robot.

- Create a ROS workspace.

  .. code:: bash

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/catkin_ws/devel/setup.bash

- Install ROS pacakge and dependencies for rplidar.

  .. code:: bash

    cd ~/catkin_ws/src
    git clone https://github.com/UCR-Robotics/rplidar_ros
    cd ~/catkin_ws
    catkin_make
    rosrun rplidar_ros create_udev_rules.sh

- Then please **replug the USB cable** for rplidar. 
  If you check USB rules by the following command, 
  you may see something like kobuki is mapped to ttyUSB0 and rplidar is mapped to ttyUSB1.

  .. code:: bash

    ls -l /dev | grep ttyUSB

- Install ROS package and dependencies for Astra Pro camera.

    sudo apt install ros-$ROS_DISTRO-rgbd-launch ros-$ROS_DISTRO-libuvc ros-$ROS_DISTRO-libuvc-camera ros-$ROS_DISTRO-libuvc-ros
    cd ~/catkin_ws/src
    git clone https://github.com/UCR-Robotics/ros_astra_camera
    cd ros_astra_camera
    ./scripts/create_udev_rules
    cd ~/catkin_ws
    catkin_make


ROS Network Setup
-----------------

- On your VM, setup environment variables in your ``.bashrc``.
  Please replace ``.21`` IP with the actual one on your robot,
  and replace ``.119`` IP with the actual one on your VM.

  .. code:: bash

    cd
    echo "export ROS_MASTER_URI=http://10.40.2.21:11311" >> .bashrc
    echo "export ROS_IP=10.40.2.119" >> .bashrc

- Please make sure there is one and only one line of code related 
  to ``ROS_MASTER_URI`` and ``ROS_IP``, respectively, appended 
  to your ``.bashrc`` file. Otherwise you will get errors. 
  You may open ``.bashrc`` file by ``gedit`` and double check this.


- Now open a new terminal and remote login to your robot with ``-X`` flag.
  You need this ``-X`` flag since you may need to open ``gedit``.

  .. code:: bash

    ssh -X ee144-nuc01@10.40.2.21

- Repeat the same steps on your robot. However, this time ROS_IP
  should be the IP address of your robot, which is the same as ROS_MASTER.

  .. code:: bash

    cd
    echo "export ROS_MASTER_URI=http://10.40.2.21:11311" >> .bashrc
    echo "export ROS_IP=10.40.2.21" >> .bashrc

- Please also make sure there is no repeated setup code in your ``.bashrc``.

- Then close all the terminals.

- With the above steps, we have basically set up an ROS environemnt
  directing all nodes on my local computer to the remote ROS master 
  on the robot.

- You may check the environemnt variables in your terminal by either of 
  the following commands.

  .. code:: bash

    echo $ROS_MASTER_URI
    echo $ROS_IP

  .. code:: bash

    env | grep ROS


Launch robot and sensors
------------------------

- Let's add a couple launch files to your local computer and robot.

- On your VM, add a launch file for rviz.

  .. code:: bash

    roscd ee144f19/launch
    touch rviz.launch
    gedit rviz.launch

- Copy and paste the following code, save and close it.

  .. code:: html

    <launch>

    <node name="rviz" pkg="rviz" type="rviz"/>

    <!--node name="rviz" pkg="rviz" type="rviz" args="-d $(find ee144f19)/rviz/nav.rviz" /-->

    </launch>

- Add another launch file for robot sensors. 
  (We do not need this on VM actually. Will copy to robot later on.)

  .. code:: bash

    roscd ee144f19/launch
    touch turtlebot_bringup_sensors.launch
    gedit turtlebot_bringup_sensors.launch

- Copy and paste the following code, save and close it.

  .. code:: html

    <launch>

    <include file="$(find turtlebot_bringup)/launch/minimal.launch" />

    <include file="$(find rplidar_ros)/launch/rplidar.launch" />

    <include file="$(find astra_camera)/launch/astrapro.launch" />

    <node pkg="tf" type="static_transform_publisher" name="footprint_to_base" args="0 0 0 0 0 0 base_footprint base_link 100" />

    <node pkg="tf" type="static_transform_publisher" name="base_to_laser" args="0 0 0.2 0 0 0 base_link laser 100" />

    <node pkg="tf" type="static_transform_publisher" name="base_to_camera" args="0 0 0.3 0 0 0 base_link camera_link 100" />

    </launch> 

- Copy your ``ee144f19`` package to your robot.

  .. code:: bash

    roscd ee144f19/..
    scp -r ee144f19 ee144-nuc01@10.40.2.21:~/catkin_ws/src

- Remote login to your robot with ``-X`` flag and compile the package you just copied.

  .. code:: bash

    ssh -X ee144-nuc01@10.40.2.21
    cd ~/catkin_ws
    catkin_make

- Finally, launch sensors on your robot. 
  (This should be done on your robot, after SSH.)

  .. code:: bash

    roslaunch ee144f19 turtlebot_bringup_sensors.launch

- You can open a new terminal on your local computer and run ``rviz`` 
  to see your robot and sensor data displayed.
  It works now because your local ROS is connected to the remote ROS on your robot.

  .. code:: bash

    roslaunch ee144f19 rviz.launch

- You can also open a new terminal on your local computer to 
  teleop your robot and take it around. 
  It will send commands to the remote computer on your robot.

  .. code:: bash

    roslaunch turtlebot_teleop keyboard_teleop.launch

.. note::
  
  If you have seen this error on your terminal, 
  it means that you didn't set up your environment variables properly.
  Please go back and check your ROS_IP and ROS_MASTER_URI 
  on both your local computer and the robot.

  .. code:: bash

    Couldn't find an AF_INET address for [ee144-nuc01]


More on RViz
------------

- RViz is a useful tool for visualization built on top of ROS. 

- You can add robot model to rviz, as well as laser scan and point cloud data.
  Play with it and you can find more interesting things!

- After your customization, you can save your rviz config file to ``ee144f19/rviz``
  folder. Then you can change the rviz launch file to use this configuration every time.
  Specifically, you can comment out the first line and uncomment the second line
  in the rviz launch file.

