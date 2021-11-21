Lab 8: Go! Turtlebot!
=====================

Overview
--------

In this lab, we will put everything together and apply what we have learned so far 
on real robots. 
The task is to navigate in a real world environment without colliding with obstacles
and finally kick the ball to the gate. Basic steps are as follows.

#. Run A* algorithm to search for an optimal path and return the list of waypoints.
#. Use the obtained waypoints to generate polynomial trajectories for the robot to follow. 

We will not provide starter code in this lab. 
Please reuse the scripts in Lab 6 and Lab 7 and combine them
into a single file named ``turtlebot.py`` for submission. 

**A successful demo on Gazebo is required before any 
implementation on the real robot**


Submission
----------

#. No Submission Required

#. Demo: **required on Gazebo and the real robot**

#. Due time: lab session

#. Grading rubric:

   - \+ 60%  Demo the task on Gazebo
      - \+30% Obtain the correct waypoints (Please print your final path).
      - \+30% Obtain an optimal trajectory based on the obstacles positon.
   - \+ 40%  Demo that the task on the real robot
      - \+10% Communicate successfully with the real robot
      - \+10% Navigate via the correct waypoints.
      - \+10% Avoid collision with obstacles.
      - \+5% Reach the goal area.
      - \+5% Kick the ball.

Lab Rules
---------

#. Safety is always the top priority.

   - No food or beverage allowed in the lab.
   - Report any suspicious cables, wires, etc.

#. Organize your station before you leave.

   - **Cut off all power supply (both robot base and NUC)**.
   - Organize wires, cables, etc.

#. Do not leave your personal information on the robot.

   - Create your own folder when you work, and delete code when you leave.
   - The robot is shared by two lab sections.

#. Do NOT make any changes to the wiring on the robot.

#. Please save the battery (recharging takes time), 
   and charge the robot if you do not have it running.

Network Setup
-------------

- If you are using native Linux OS or dual boot with Linux OS, 
  you can skip this section. 
  (As long as you can connect to the robot with your network card,
  then you don't need the USB WiFi Adapter.)
  
-  In order to have bi-directional communications between your laptop and the robot, we use a USB WiFi apapter binding your VM to a pysical MAC address, rather than sharing the network of your host computer. Open your Ubuntu VM, follow the following instructions to install Linux driver for USB WiFi Adapter

   -  Open a new terminal, download the driver and install it by the following commands.
   
      .. code-block:: bash

       cd ~/Downloads
       wget https://github.com/UCR-Robotics/ee144/raw/wifi-adapter-driver/RTL88x2BU_WiFi_linux_v5.3.1.zip
       unzip RTL88x2BU_WiFi_linux_v5.3.1.zip
       cd RTL88x2BU_... [press Tab key to complete]
       chmod +x install.sh
       sudo ./install.sh
    
   -  Restart your computer by one more command.
   
      .. code-block:: bash

       sudo reboot
    
   -  Plug in your adapter. If you can see the flickering blue light on your adapter, then you are good.
   
- Shutdown your VM. 
  Go to the settings of your VM, in ``Network Adapter`` section, 
  uncheck ``Connect at power on``, 
  and select ``NAT`` as the network connection.
  
- Go to the ``USB Controller`` section of the settings, 
  select ``USB 3.0`` as the USB compatibility.
  
- Plug in (USB 3.0 port if applicable) the USB WiFi adapter on your host computer,
  then pass (this USB device) into your VM. 
  (You can find options on the menubar to pass removable devices into VM.)
  
.. note::

  Sometimes the system upgrade may disable the linux driver.
  The solution would be that just install the driver again.
  
Remote Login
------------

- Connect to ``roboticslab`` WiFi network. 
  Please ask TAs for credentials.
  Check if your Internet connection is good.

- The IP address of your Ubuntu VM is dynamically allocated, 
  while the IP address of your robot (NUC computer) is static.

- Connect power bank to the NUC onboard computer on your robot, 
  then turn on the robot and NUC computer.

- To remote login to the NUC computer on your robot, 
  open a new terminal and run

  .. code-block:: bash

    ssh username@NUC_IP

- Replace the above ``username`` and ``NUC_IP`` with the actual one.
  For example, the IP address of robot 01 is ``10.40.2.21``, 
  and the IP address of robot 02 is ``10.40.2.22``, and so on.
  The username on NUC computer is ``ee144-nuc01`` for robot 01, and so on.

- For example, for robot 01 we can use

  .. code-block:: bash

    ssh ee144-nuc01@10.40.2.21

- Please ask TAs for the password of this account.

- You can see the new username and hostname on your terminal if you succeed.
  It should be like ``ee144-nuc01@ee144-nuc01``.

- If you want to use graphic tools later on, then use

  .. code-block:: bash

    ssh -X username@NUC_IP   (must be capitalized X)

- To disconnect, just run

  .. code-block:: bash

    exit

- To shutdown your remote computer, run

  .. code-block:: bash

    sudo shutdown now
    
Copy Files
----------

- Command ``scp`` (secure copy) can help you copy files between two computers.
  
- To copy files from your VM to robot, open a terminal in your VM and run

  .. code-block:: bash

    scp /path/to/file/name.py username@NUC_IP:/path/to/destination

- To copy files from robot to your VM, just switch the above two arguments

  .. code-block:: bash

    scp username@NUC_IP:/path/to/file/name.py /path/to/destination 
    
Communication with TurtleBot
----------------------------

- Once you have successfully login to the actual robot, 
  the following command can bring up the Kobuki mobile base. 

  .. code-block:: bash
    
    roslaunch turtlebot_bringup minimal.launch --screen

- Then you can open another terminal and remote login (again, twice) to the robot to run the script.

- Alternatively, you can use another terminal to run the teleop command for testing **using the default linear and angular velocity**. 

  .. code-block:: bash
    
    roslaunch turtlebot_teleop keyboard_teleop.launch

- To edit the script already copied to the robot, use the following command. 
  (This is where you may fail if ``-X`` option was not specified when using ssh.)

  .. code-block:: bash
    
    gedit ~/team01/turtlebot.py

- Then demo to TAs.

.. note::

  When you bring up the robot, the odometry will be reset (initialized to origin).

About the implementation
------------------------

- We divide the space into three parts to accommodate three teams at the same time.
  They are designed to have different layouts. 
  
- Each team have unlimited trials during the allocated time slot for the final demo on the real robot.

- For each trial, the robot will start from one of the six starting grids, plan and follow its smooth trajectory, kick the ball, and stop at the goal area.

- The robot should have a reasonable velocity in order to kick the ball and not collides with the wall.


.. Field Map
 ---------
 .. image:: pics/capstone_map.jpg
  :width: 80%
  :align: center
  
 - **The grid size is 0.5m**, which is slightly larger than the size of the robot.

 - The grey grids are obstacles and walls that the robot should not collide with.

 - The six green grids on the bottom right corner are starting areas. 
  For each trial during the demo, one of them will be picked at random. 
  (You will be informed which grid to start before you run the script.)

 - On the top side, the red grid is the goal area where the robot should stop, 
  and the orange grid is the buffer area where the robot should pass through, in order to kick the ball.

 - On the top side, the narrow gate is marked by dark blue color,
  and the wide gate is marked by light blue color.

 - The ball is placed on the common edge of the orange and red grid, marked by dark green color. 
