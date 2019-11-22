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

  .. code:: bash

    rosrun rplidar_ros create_udev_rules.sh

- Then please **replug the USB cable** for rplidar. 
  If you check USB rules by the following command, 
  you may see something like kobuki is mapped to ttyUSB0 and rplidar is mapped to ttyUSB1.

  .. code:: bash

    ls -l /dev | grep ttyUSB

- Install dependencies for Astra Pro camera.

  .. code:: bash

    sudo apt-get update
    sudo apt install ros-$ROS_DISTRO-rgbd-launch ros-$ROS_DISTRO-libuvc ros-$ROS_DISTRO-libuvc-camera ros-$ROS_DISTRO-libuvc-ros

.. note::

  Please make sure you have done the above two steps without any error messages.
  If you experience issues connecting to the keyserver (an example error shown below), 

  .. code::

    The following signatures couldn't be verified because the public key is not available: NO_PUBKEY F42ED6FBAB17C654

  you can go to `ROS installation webpage <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_
  and run step ``1.3 Set up your keys``, and then try the above two steps again.

- Install the ROS package for Astra Pro camera. 

  .. code:: bash

    cd ~/catkin_ws/src
    git clone https://github.com/UCR-Robotics/ros_astra_camera
    cd ros_astra_camera
    ./scripts/create_udev_rules
    cd ~/catkin_ws
    catkin_make

- Then please **replug the USB cable** for Astra camera.


ROS Network Setup
-----------------

- On your VM, setup environment variables in your ``.bashrc``.
  Please replace ``.21`` IP with the actual one on your robot,
  and replace ``.119`` IP with the actual one on your VM.

  .. code:: bash

    echo "export ROS_MASTER_URI=http://10.40.2.21:11311" >> ~/.bashrc
    echo "export ROS_IP=10.40.2.119" >> ~/.bashrc
    source ~/.bashrc

- Please make sure there is one and only one line of code related 
  to ``ROS_MASTER_URI`` and ``ROS_IP``, respectively, appended 
  to your ``.bashrc`` file and the IPs are correct. Otherwise you will get errors. 
  You may open ``.bashrc`` file by ``gedit`` and double check this.

- Now open a new terminal and remote login to your robot with ``-X`` flag.
  You need this ``-X`` flag since you may need to open ``gedit``.

  .. code:: bash

    ssh -X ee144-nuc01@10.40.2.21

- Repeat the same steps on your robot. However, this time ROS_IP
  should be the IP address of your robot, which is the same as ROS_MASTER.

  .. code:: bash

    echo "export ROS_MASTER_URI=http://10.40.2.21:11311" >> ~/.bashrc
    echo "export ROS_IP=10.40.2.21" >> ~/.bashrc
    source ~/.bashrc

- Please also make sure there is no repeated setup code in your ``.bashrc``.

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

.. note::

  If you do not want to work with robot in this way later on
  (e.g., just run Gazebo locally), you need to delete or comment out the last
  two lines of code of ROS_MASTER_URI and ROS_IP in your ``.bashrc``,
  because keeping these two lines means that you are trying to connect
  to a ROS master on the robot. 
  When you work offline/locally, you do not have the connection to robot.

  Please also remember to do ``source ~/.bashrc`` to take effect, 
  or close all terminals and try it again with new terminals, 
  since ``.bashrc`` will only be executed for once when you open a new terminal.


Launch Robot and Sensors
------------------------

- Let's add a couple launch files to your local computer and robot.

- On your VM, add a launch file for rviz.

  .. code:: bash

    roscd ee144f19/launch
    touch rviz.launch
    gedit rviz.launch

- Copy and paste the following code, save and close it.

  .. literalinclude:: ../launch/rviz.launch
    :language: xml

- Add another launch file for robot sensors. 
  (We do not need this on VM actually. Will copy to robot later on.)

  .. code:: bash

    roscd ee144f19/launch
    touch turtlebot_bringup_sensors.launch
    gedit turtlebot_bringup_sensors.launch

- Copy and paste the following code, save and close it.

  .. literalinclude:: ../launch/turtlebot_bringup_sensors.launch
    :language: xml

- Copy your ``ee144f19`` package to your robot.

  .. code:: bash

    roscd ee144f19/..
    scp -r ee144f19 ee144-nuc01@10.40.2.21:~/catkin_ws/src

- Remote login to your robot with ``-X`` flag and compile the package you just copied.

  .. code:: bash

    ssh -X ee144-nuc01@10.40.2.21
    cd ~/catkin_ws
    catkin_make

- Finally, launch robot base and sensors on your robot. 
  (This should be done on your robot, after SSH.)

  .. code:: bash

    roslaunch ee144f19 turtlebot_bringup_sensors.launch

.. note::
  
  Sometimes ROS cannot find the new copied package. 
  If you cannot auto-complete the above command, 
  you can ask ROS to search new packages again in existing workspace
  by the following command.

  .. code:: bash

    rospack profile

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

  .. code::

    Couldn't find an AF_INET address for [ee144-nuc01]

  The ROS_MASTER_URI on both machines should be the same, all pointing towards your robot.
  The ROS_IP should be different. It should be the actual IP address of the machine.


More on RViz
------------

- RViz is a useful tool for visualization built on top of ROS. 
  Play with it and you can find more interesting things!

- Since you don't have the map available right now, you may want to first change
  the ``Fixed Frame`` in ``Global Option`` to be ``odom``.

- You can add laser scan and point cloud data to RViz.
  For example, select ``Add`` on the bottom left corner of the window. 
  You can pick data type ``LaserScan`` or ``PointCloud2``.
  Then on the left side bar, you need to manually choose the topic you want to display
  for LaserScan or PointCloud2.

- Alternatively, you can click ``Add`` and switch to ``By Topic`` tab, 
  and select ``/camera/depth/points/PointCloud2`` or ``/scan/LaserScan``.

- You can view the real time images of RGB camera by add the topic ``/camera/rgb/image_raw/Image``.

- You can also add a robot model to rviz, to show where your robot is.  

- After your customization, you can save your rviz config file to ``ee144f19/rviz``
  folder. Maximize the RViz window, then you can see ``file`` on the manubar.
  Select ``Save Config As`` and save it to ``ee144f19/rviz`` with the name ``nav.rviz`` .
  
- Then you can change the rviz launch file to use this configuration every time.
  Specifically, you can comment out the first line and uncomment the second line
  in the rviz launch file. The launch file is ``ee144f19/launch/rviz.launch``

  .. code:: xml

    <launch>

      <!--node name="rviz" pkg="rviz" type="rviz"/-->

      <node name="rviz" pkg="rviz" type="rviz" args="-d $(find ee144f19)/rviz/nav.rviz" />

    </launch>


.. note:: 

  When multiple ROS nodes from different machines connecting to the same ROS master
  on one of the machines, you may experience issues with time stamps of the messages sent between each other.
  In this case, you have to make sure that all the clocks on these machines are synchronized.
  
  If not, the behavior would be like, a message sent from machine A to machine B with a time stamp
  11:00am. However, machine B is five minutes late compared with machine A, i.e. 10:55am when machine A sent the message. 
  Then the message will display on machine B's RViz 5 minutes later.
