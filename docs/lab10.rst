Lab 10: Navigation Stack
========================

Overview
--------

In this lab, we will introduce the laser-based SLAM algorithm provided by 
`ROS Navigation Stack <http://wiki.ros.org/navigation>`_. 

You will get familiar with the core concepts/modules as shown in the picture below.

.. image:: pic/nav_stack.png

Submission is not required, since this lab is designed just for your information.


Mapping
-------

- `slam_gmapping <http://wiki.ros.org/gmapping>`_ is a ROS package that
  can create a 2-D occupancy grid map from laser and pose data collected by a mobile robot.

- Please first make sure you have ``ROS_MASTER_URI`` and ``ROS_IP`` setup correctly on both your VM
  and the robot.

- To create a map using ``gmapping``, you need to open five terminals: 
  three on the robot to run algorithms, and two on VM to send remote control commands and show visualization.

- The first one: ``ssh`` to the robot, and bringup the base and sensors, as what we did in Lab 8.

  .. code:: bash

    ssh -X ee144-nuc01@10.40.2.21
    roslaunch ee144f19 turtlebot_bringup_sensors.launch

- The second one: run ``slam_gmapping`` node to build up the map. 
  This algorithm will match and update the map from newcoming laser data.
  We need to also add constrains on the size of the map.

  .. code:: bash

    ssh -X ee144-nuc01@10.40.2.21
    rosrun gmapping slam_gmapping scan:=/scan _xmin:=-5 _xmax:=5 _ymin:=-5 _ymax:=5

- The third one: run keyboard teleop to take the robot move around the room and build up the map.

  .. code:: bash

    roslaunch turtlebot_teleop keyboard_teleop.launch

- The fourth one: run ``rviz`` locally to see the map building process and adjust teleop commands accordingly. 

  .. code:: bash

    roslaunch ee144f19 rviz.launch

- You need to add two visualization modules in rviz. One is the map, the other is the robot. 
  Just click ``Add`` and find ``Map`` and ``RobotModel``. 
  Then you can see where the robot is on the map.

- You can then move the robot around until the map is in good shape.
  Then you can open the fifth terminal and save the map.

  .. code:: bash

    ssh -X ee144-nuc01@10.40.2.21
    roscd ee144f19
    mkdir map
    cd map
    rosrun map_server map_saver -f wch109

- Now you should be able to see the saved map ``wch109.pgm`` and ``wch109.yaml``.
  Then you can close all the terminals.


Navigation Stack Setup
----------------------

- The following setup tutorials are modified from `ROS Navigation Stack tutorials 
  <http://wiki.ros.org/navigation/Tutorials/RobotSetup>`_.
  All the following steps should be on the robot.

- Open a new terminal, remote login to the robot, and go to your ``ee144f19`` package. 
  We need to set up some parameters for navigation stack.

  .. code:: bash

    ssh -X ee144-nuc01@10.40.2.21
    roscd ee144f19
    mkdir config
    cd config
    touch base_local_planner_params.yaml
    gedit base_local_planner_params.yaml

- Please copy and paste the parameters for local planner, then save and close it.

  .. literalinclude:: ../config/base_local_planner_params.yaml
      :language: xml

- Create a new file for common parameters in costmap.

  .. code:: bash

    touch costmap_common_params.yaml
    gedit costmap_common_params.yaml

- Please copy and paste the following parameters, then save and close it.

  .. literalinclude:: ../config/costmap_common_params.yaml
      :language: xml
      
- Create a new file for local costmap parameters.

  .. code:: bash

    touch local_costmap_params.yaml
    gedit local_costmap_params.yaml

- Please copy and paste the following parameters, then save and close it.

  .. literalinclude:: ../config/local_costmap_params.yaml
      :language: xml

- Create a new file for global costmap parameters.

  .. code:: bash

    touch global_costmap_params.yaml
    gedit global_costmap_params.yaml

- Please copy and paste the following parameters, then save and close it.

  .. literalinclude:: ../config/global_costmap_params.yaml
      :language: xml

- Let's then switch to the launch file.

  .. code:: bash

    cd ../launch
    touch move_base.launch

- Then copy and paste the following.

  .. literalinclude:: ../launch/move_base.launch
      :language: xml


Navigation Stack
----------------

- Up to this point, you have everything you need for autonomous navigation demo.

- We need three terminals: two on the robot to run the algorithm, and one on our VM to show the visualization.

- As usual, bring up robot base and sensors in the first terminal.

  .. code:: bash

    ssh -X ee144-nuc01@10.40.2.21
    roslaunch ee144f19 turtlebot_bringup_sensors.launch

- Run navigation stack on the second terminal.

  .. code:: bash

    ssh -X ee144-nuc01@10.40.2.21
    roslaunch ee144f19 turtlebot_bringup_sensors.launch

- Show visualization locally on the VM in the third terminal.

  .. code:: bash

    roslaunch turtlebot_rviz_launchers view_navigation.launch

- You can see a couple of interesting things in rviz. 

  - The occupancy grid map has been augmented by global costmap.
  - The obstacles around robot are augmented by local costmap.
  - You can also visualize global and local paths planned by the robot.

- Now put the robot on the ground. In rviz, you can set the ``2D Pose Estimate`` (somewhere under menubar).
  Make sure the arrow you put on the map is the actual position of the robot.
  Then you can see the visualized robot in rviz "jumped" to the initial pose you picked.

- Lastly, the most interesting part comes. You can then click ``2D Nav Goal`` and put
  another arrow somewhere on the ground. 
  You will see the robot move from its currenct position to the goal position autonomously.

- Have fun!







