Lab 8: Go! Turtlebot!
=====================

Overview
--------

In this lab, we will put everything together and apply what we have learned so far 
on real robots. 
The task is to navigate in a real world environment without colliding with obstacles
and finally kick the ball to the gate. Basic steps are as follows.

#. Decompose the real world environment into a grid map.
#. Transform the grid map into a graph with nodes and weights assigned.
#. Run A* algorithm to search for an optimal path and return the list of waypoints.
#. Generate polynomial trajectories for the robot to follow. 

We will not provide starter code in this lab. 
Please reuse the scripts in Lab 6 and Lab 7 and combine them
into a single file named ``turtlebot.py`` for submission.


Submission
----------

#. Submission: group submission via Gradescope

#. Demo: **required**

#. Due time: 11:59pm, Dec 12, Saturday

#. Files to submit:

   - lab8_report.pdf
   - turtlebot.py

#. Grading rubric:

   + \+ 50%  Clearly describe your approach and explain your code in the lab report.
   + \+ 50%  Demo that the robot is able to move from the start grid to the goal grid 
     without colliding with obstacles.
   + \- 15%  Penalty applies for each late day. 


VPN Connection
--------------

To make sure you can have access to the remote server, please double check the following items.

#. You have an active ENGR account. Recall the account name and password in your mind. 
   You may try to login `at this page <https://intra.engr.ucr.edu/apps/systems/login.php>`_.

#. You can connect to ENGR VPN using ENGR account and 
   `Cisco AnyConnect <https://systems.engr.ucr.edu/help/vpn>`_.

#. You have a terminal to run the remote login command. 
   You can use the terminals in VM (as in all previous labs), 
   or directly connect from the host computer (use native terminals in Mac; or
   install `MobaXterm <https://mobaxterm.mobatek.net/download-home-edition.html>`_ in Windows).

#. Connect to ENGR VPN, open a new terminal and run command ``ping gauss`` to check the connection. 
   You should see responses if you have successfully connected to ENGR VPN, 
   otherwise please double check VPN connection.

#. Run command ``ping ee144-nuc11`` to check the connection to the remote login server. 
   If you can see responses, you are good and no further action needed. 
   Otherwise, **please send your ENGR account name to TA**, 
   who can work with the system manager to grant you access.


Remote Login
------------

- We need to first connect to a remote Ubuntu server and then connect to robots from this server.
  This remote server serves as a bridge to help us jump from engineering network to the actual roboticslab network.
  
- After connecting to ENGR VPN, open a new terminal and run the following command to login to the Ubuntu server.
  We will talk about the password during lab hours.

  .. code-block:: bash

    ssh -X ee144-remote@ee144-nuc11    # -X can enable graphics support; e.g., to open gedit

- Create a folder to organize files for your team. You only need to do this once.

  .. code-block:: bash

    mkdir team01   # use your actual team number

- From ``nuc11`` computer, remote login again to one of the three robots. 
  (We use robot #1, #3 and #6; the corresponding IP addresses are 21, 23 and 26.)

  .. code-block:: bash

    ssh -X ee144-remote@10.40.2.21
    ssh -X ee144-remote@10.40.2.23
    ssh -X ee144-remote@10.40.2.26

- Again, create a folder for your team. 

  .. code-block:: bash

    mkdir team01

- To disconnect, just run ``exit`` in the terminal. 


Copy Files
----------

- Command ``scp`` (secure copy) can help you copy files between two computers.
  
- To copy files from your local computer to the robot, open a terminal and run

  .. code-block:: bash

    scp path/to/filename.py username@host_ip:/path/to/destination

- To copy files from robot to your VM, just switch the above two arguments

  .. code-block:: bash

    scp username@host_ip:path/to/filename.py /path/to/destination 

- Since we need to remote login twice, the files also need to be copied twice,
  where the remote server can relay the files to the robots.
  An example is provided as follows.

  .. code-block:: bash

    scp turtlebot.py ee144-remote@ee144-nuc11:~/team01/turtlebot.py
    ssh -X ee144-remote@ee144-nuc11
    scp ~/team01/turtlebot.py ee144-remote@10.40.2.21:~/team01/turtlebot.py

- Another option is to use FileZilla. For Windows and MacOS laptops, you can 
  `download FileZilla here <https://filezilla-project.org/download.php?show_all=1>`_.

- For linux laptop, run the following command to install.

  .. code-block:: bash
    
    sudo apt install filezilla

- If you are using MobaXterm in Windows, it also supports file transfer. 


Bringup TurtleBot
-----------------

- Once you have successfully login to the actual robot, 
  the following command can bring up the Kobuki mobile base. 

  .. code-block:: bash
    
    roslaunch turtlebot_bringup minimal.launch --screen

- Then you can open another terminal and remote login (again, twice) to the robot to run the script.

- Alternatively, you can use another terminal to run the teleop command for testing. 

  .. code-block:: bash
    
    roslaunch turtlebot_teleop keyboard_teleop.launch

- To edit the script already copied to the robot, use the following command. 
  (This is where you may fail if ``-X`` option was not specified when using ssh.)

  .. code-block:: bash
    
    gedit ~/team01/turtlebot.py

- We highly recommend that you test the scripts locally before sending to the robot,
  as the debugging experience over remote ssh connections could be cumbersome.


Field Map
---------

.. image:: pics/capstone_map.jpg
  :width: 80%
  :align: center

- We divide the space into two parts to accommodate two teams at the same time.
  They are designed to have exactly the same layout. 
  
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
