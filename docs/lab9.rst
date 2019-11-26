Lab 9: Motion Capture System
============================

Overview
--------

A motion capture system typically consists of multiple high resolution infrared cameras.
Each camera can produce some reasonable amount of infrared ray to its field of view.
We will then attach some refletive markers on our robots, which can reflect infrared rays.
In the end, these infrared rays will be recaptured by the cameras again and then be used for 
3D reconstruction of the robot pose.

In this lab, we will introduce how to use motion capture system in the robotic systems.
We will go through the entire process step by step and show you how to attach markers on the robot,
why the markers have to be asymmetric, how to create the object in mocap system, and how to 
read your pose information via ROS.

Submission is not required, since this lab is designed just for your information.


Mocap Setup
-----------

- Open a new terminal and go to your ``ee144f19`` package. 
  We need to add two more files.

  .. code:: bash

    roscd ee144f19/launch
    touch robots.yaml
    gedit robots.yaml

- Please copy and paste the configuration for objects used in mocap.
  If you want, you can keep the one for your robot and delete others.

  .. literalinclude:: ../launch/robots.yaml
      :language: yaml

- Add the launch file for OptiTrack motion capture system.

  .. code:: bash

    touch mocap.launch
    gedit mocap.launch
  
- Please copy and paste the code, then save and close it.

  .. literalinclude:: ../launch/mocap.launch
      :language: xml

- Copy this package to robot.

  .. code:: bash

    roscd ee144f19/..
    scp -r ee144f19 ee144-nuc01@10.40.2.21:~/catkin_ws/src

- Now remote login to your robot. Download and install the mocap ROS package.

  .. code:: bash

    ssh -X ee144-nuc01@10.40.2.21
    cd ~/catkin_ws/src
    git clone https://github.com/UCR-Robotics/mocap_optitrack
    cd ..
    catkin_make

- All set. Now you can launch motion capture software and see the pose feedback of your robot. 
  You can launch it by the following command.

  .. code:: bash

    roslaunch ee144f19 mocap.launch

.. note::
  
  Sometimes ROS cannot find the copied new package. 
  If you cannot auto-complete the above command, 
  you can ask ROS to search new packages again in existing workspace
  by the following command.

  .. code:: bash

    rospack profile

- You can check your robot pose by the command. 
  Please replace ``robot01`` with the actual one.

  .. code:: bash

    rostopic echo /mocap_node/robot01/pose2d


Changes in the Script
---------------------

- Recall the script we have been using since lab3. 
  If you want to replace ``odom`` feedback with ``mocap`` feedback,
  you need to make changes in two places. 
  First, add one more line to subscribe the topic from ``mocap_node``
  and comment out the odom one. 
  Remember to replace the robot number with the actual one you are using.
  Second, add one more callback function to read and store pose2D data.
  You can put the ``mocap_callback`` function after ``odom_callback`` function.

  .. code:: python

    class Turtlebot():
        def __init__(self):
            rospy.init_node("turtlebot_move")
            rospy.loginfo("Press CTRL + C to terminate")

            self.pose = Pose2D()
            # self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)  # comment this out
            self.mocap_sub = rospy.Subscriber("mocap_node/robot01/pose2d", Pose2D, self.mocap_callback)  
            # remember to replace robot number with the actual one you are using

            # some other code here


        def odom_callback(self, msg):
            # some code here


        def mocap_callback(self, msg):
            self.pose.theta = msg.theta
            self.pose.x = msg.x
            self.pose.y = msg.y

            # Logging once every 100 times
            self.logging_counter += 1
            if self.logging_counter == 100:
                self.logging_counter = 0
                self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
                rospy.loginfo("odom: x=" + str(self.pose.x) +\
                    ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))

.. note::

  You have to be aware of the difference between mocap frame and odom frame, in order to 
  use it correctly/properly. Let's denote mocap frame as the world frame :math:`W`, 
  denote odom frame as :math:`O`, and denote robot frame as :math:`R`.

  - Whenever the robot wakes up, it will set the current wake-up point as the origin of ``odom``
    frame, and set x and y axes accordingly. After initialization, the wheel odometry will publish
    the transformation between current pose and the initial odom pose (origin pose), 
    which is :math:`T_{OR}`. If you want to know the robot pose with respect to the world frame,
    you need to first figure out the fixed transformation :math:`T_{WO}` (i.e. where the robot was initialized),
    and then multiply them to get :math:`T_{WR}`.

  - If using motion capture system, you will directly get the robot pose in world frame.
    In other words, the pose you get from ``mocap_node`` topic will always be :math:`T_{WR}`.
    This is useful when you do not want to consider different initial positions of your robot.
    For example, you can store the map (and write the algorithm) in world frame, and then for each starting point, 
    you only need to specify which grid it is in world frame, 
    rather than shift the entire map with respect to the initial point of odom frame.

