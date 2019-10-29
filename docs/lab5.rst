Lab 5: Go! TurtleBot!
=======================

Overview
--------

In this lab, we will (finally) work on real TurtleBot robots.
Please work in groups of two and share one robot. 
The goal of this lab is to help you get familiar with this TurtleBot robot,
and apply your closed-loop control sctipts on the robot.
The demo will be driving the robot to move along the edges of a 
2m L x 0.5m W rectangle and return to the origin, 
using closed-loop control (as we did in Lab 3).
Please note that you may need to make some changes to your code,
as the code working in simulation does not necessary work on real robot very well.

Submission
----------

- Submission: team submission via iLearn, due at the beginning of next lab

- Demo: required, due by the end of next lab

- Files to submit: **(please do not zip, just upload all files)**

  #. lab5_report_01.pdf (replace 01 with your team number, **use number from 01-18**)
  #. lab5_closed_loop.py
  
- Grading rubric:

  + \+ 50%  Clearly describe your approach and explain your code in lab report.
  + \+ 30%  Be able to make the robot move.
  + \+ 20%  Apply closed-loop control on the robot and demo to TA.
  + \- 15%  Penalty applies for each late day. 

Preview: We will work on waypoint navigation (motion planning) next time.


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

#. Please find your station according to your team number. 

   - In Tuesday lab, your station/robot number is your team number.
   - In Thursday lab, your station/robot number is your team number minus 10.

#. Please save the battery (recharging takes time), 
   and charge the robot if you do not have it running.


Connect to Robot
----------------

- Go to the settings of your VM, in ``Network Adapter`` section, 
  uncheck ``Connect at power on``, and select ``NAT`` as the network connection.

- Go to the ``USB Controller`` section of the settings, 
  select ``USB 3.0`` as the USB compatibility.

- Open your Ubuntu VM, `follow the instructions here <reference.html#usb-wifi-adapter>`_
  and install Linux driver for USB WiFi Adapter.

- Plug in (USB 3.0 port if applicable) the USB WiFi adapter on your host computer,
  then pass (this USB device) into your VM.

- Connect to ``roboticslab`` WiFi network. Please ask TA for credentials.
  Check if your Internet connection is good.

- Connect power bank to the NUC onboard computer on your robot, 
  then turn on the robot and NUC computer.

- The IP address of your Ubuntu VM is dynamically allocated, 
  while the IP address of your robot (NUC computer) is static. 
  For example, the IP address of robot01 is ``10.40.2.21``, 
  and the IP address of robot02 is ``10.40.2.22``, and so on.
  The username on NUC computer is ``ee144-nuc01`` for robot01, and so on.

- To remote login to the NUC computer on your robot, 
  open a new terminal and try (make sure you are in roboticslab network)

  .. code:: bash

    ssh username@NUC_IP

- If you want to use graphic tools later on, then use

  .. code:: bash

    ssh -X username@NUC_IP   (must be capitalized X)

- For example, for robot01 we can use

  .. code:: bash

    ssh -X ee144-nuc01@10.40.2.21

- Please ask TA for the password of this account.

- You can see the new username and hostname on your terminal if you succeed.
  It should be like ``ee144-nuc01@ee144nuc01``.

- To disconnect, just run

  .. code:: bash

    exit


Copy Files
----------

- Command ``scp`` (secure copy) can help you copy files between two computers.
  
- To copy files from your VM to robot, open a terminal in your VM and run

  .. code:: bash

    scp /path/to/file/name.py username@NUC_IP:/path/to/destination

- To copy files from robot to your VM, run from the terminal on NUC
  (you need to figure out the username and IP address of your VM first)

  .. code:: bash

    scp /path/to/file/name.py VM_username@VM_IP:/path/to/destination

- Another option is using FileZilla. For Windows and MacOS laptops, you can 
  `download FileZilla here <https://filezilla-project.org/download.php?show_all=1>`_.

- For linux laptop, run the following command to install.

  .. code:: bash
    
    sudo apt install filezilla


Bringup TurtleBot
-----------------

- Turn on the robot and NUC computer, make sure the wiring on robot is good.
  Open a new terminal in VM, remote login into your robot, and run

  .. code:: bash
    
    roslaunch turtlebot_bringup minimal.launch --screen

- You should hear some sound here if you succeed.

- Copy your code to robot and try to make it run in a rectangle. 
  **Then demo to TA**.


Example
-------

- For example, open a new terminal in your VM and go to your ROS package.

  .. code:: bash

    roscd ee144f19/scripts
    cp closed_loop_square_p_ctrl.py lab5_closed_loop.py 
    gedit lab5_closed_loop.py

- Then make some changes (dimension of square, etc.) 
  and make sure this script works in Gazebo simulation.

- Open a new terminal, and remote login to your robot and create a new folder.

  .. code:: bash
    
    ssh -X ee144-nuc01@10.40.2.21
    [enter password]

    mkdir team01

- Go back to previous terminal, copy this script to your robot.

  .. code:: bash
    
    scp ./lab5_closed_loop.py ee144-nuc01@10.40.2.21:~/team01/

- You can use the terminal running remote login session to open file manager
  or gedit editor (works only if you login with ``-X`` option)

  .. code:: bash
    
    nautilus .

    gedit filename.py

- If you want to have multiple terminals running remote login session,
  then you need to open multiple terminals in your VM and remote login respectively.

