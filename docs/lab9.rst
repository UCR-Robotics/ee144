Lab 9: Navigation Stack
=======================

Overview
--------

In this lab, we will work on the laser-based SLAM algorithm provided by 
`ROS Navigation Stack <http://wiki.ros.org/navigation>`_. 

You will get familiar with the core concepts/modules as shown in the picture below.

.. image:: pic/nav_stack.png

Please take a screenshot for each question below and answer in your lab report.

#. What is the difference (and/or relation) between ``map``, ``odom``, ``footprint``, ``base``
   and ``sensors`` coordinate frames? Please give explaination to each of them.
#. What is the difference between ``global planning`` and ``local planning``?
#. what is ``Localization`` problem? 
   Please explain how the particle filter algorithm works. (e.g., initialization, update, etc.)
#. What should be the data format for laser scanner and depth camera? What is the difference?

You may refer to your lecture slides for more details. 
However, I will cover all of them in lab sessions.

Preview: Congratulations. This is your last lab assignment. 
Next time we will work on the final capstone project.


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


Setup
-----

- Open a new terminal and go to your ``ee144f19`` package. 
  We need to set up some parameters for navigation stack.

  .. code:: bash

    roscd ee144f19
    mkdir config
    cd config
    touch base_local_planner_params.yaml
    gedit base_local_planner_params.yaml

- Please copy and paste the parameters for local planner, then save and close it.

  .. code:: html
  
    TrajectoryPlannerROS:
        max_vel_x: 0.45
        min_vel_x: 0.1
        max_vel_theta: 1.0
        min_in_place_vel_theta: 0.4

        acc_lim_theta: 3.2
        acc_lim_x: 2.5
        acc_lim_y: 2.5

        holonomic_robot: false

- Create a new file for common parameters in costmap.

  .. code:: bash

    touch costmap_common_params.yaml
    gedit costmap_common_params.yaml

- Please copy and paste the following parameters, then save and close it.

  .. code:: html
    
    obstacle_range: 2.0
    raytrace_range: 3.0
    footprint: [[0.2, 0.2], [0.2, -0.2], [-0.2, 0.2], [-0.2, -0.2]]
    #robot_radius: 0.20 #ir_of_robot
    inflation_radiuscans: 0.50

    observation_sources: laser_scan_sensor #point_cloud_sensor
    laser_scan_sensor: {sensor_frame: laser, data_type: LaserScan, topic: /scan, marking: true, clearing: true}
    #point_cloud_sensor: {sensor_frame: camera, data_type: PointCloud, topic: /camera/depth/points, marking: true, clearing: true}

- Create a new file for local costmap parameters.

  .. code:: bash

    touch local_costmap_params.yaml
    gedit local_costmap_params.yaml

- Please copy and paste the following parameters, then save and close it.

  .. code:: html

    local_costmap:
        global_frame: odom
        robot_base_frame: base_link
        update_frequency: 5.0
        publish_frequency: 2.0
        static_map: false
        rolling_window: true
        width: 4.0
        height: 4.0
        resolution: 0.05

- Create a new file for global costmap parameters.

  .. code:: bash

    touch global_costmap_params.yaml
    gedit global_costmap_params.yaml

- Please copy and paste the following parameters, then save and close it.

  .. code:: html

    global_costmap:
        global_frame: /map
        robot_base_frame: base_link
        update_frequency: 5.0
        static_map: true

- Let's then switch to the launch file.

  .. code:: bash

    cd ../launch
    touch move_base.launch

- Then copy and paste the following.

  .. code:: html

    <launch>

    <master auto="start"/>
    <!-- Run the map server --> 
    <node name="map_server" pkg="map_server" type="map_server" args="$(find ee144f19)/map/wch109.pgm 0.05"/>

    <!-- Run AMCL --> 
    <include file="$(find amcl)/examples/amcl_diff.launch" />
    <!-- include file="$(find amcl)/examples/amcl_omni.launch" /-->

    <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
        <rosparam file="$(find ee144f19)/config/costmap_common_params.yaml" command="load" ns="global_costmap" /> 
        <rosparam file="$(find ee144f19)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find ee144f19)/config/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find ee144f19)/config/global_costmap_params.yaml" command="load" /> 
        <rosparam file="$(find ee144f19)/config/base_local_planner_params.yaml" command="load" />
        <remap from="cmd_vel" to="cmd_vel_mux/input/teleop" />
    </node>

    </launch> 

- Copy your package to the robot and compile it.

  .. code:: bash

    roscd ee144f19/..
    scp -r ee144f19 ee144-nuc01@10.40.2.21:~/catkin_ws/src
    ssh ee144-nuc01@10.40.2.21
    cd ~/catkin_ws
    catkin_make



Remote Login
------------

- Please download the teamviewer host package, copy it to your robot and install.

  .. code:: bash

    cd ~/Download
    wget https://download.teamviewer.com/download/linux/teamviewer-host_amd64.deb
    scp ./teamvierew-host_amd64.deb ee144-nuc01@10.40.2.21:~/Download
    ssh ee144-nuc01@10.40.2.21
    cd ~/Download
    sudo dpkg -i teamvierew-host_amd64.deb

- Please download teamview client software on your own laptop, and then remote login to your robot
  by the corresponding IP address.


ssh-keygen -t rsa -b 4096

ssh-copy-id remote_username@server_ip_address


rm .ssh/known_hosts
ssh username@hostname -oHostKeyAlgorithms='ssh-rsa'

 sudo apt-get -f install ./teamviewer-host_amd64.deb

Couldn't find an AF_INET address for [ee144-nuc11]
