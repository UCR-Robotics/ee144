Lab 2: Open Loop Control
========================

Overview
--------

In this lab, we are going to write a script to make TurtleBot follow a square trajectory.
You are supposed to use an "open-loop" control approach. 
The side length of the square is set to be 4m. 
Specifically, the robot starts from its origin, moves forward 4 meters, then turns 90 degrees,
moves forward 4 meters, turns again, and so on, until goes back to its origin. 
(It's up to you to make a left turn or right turn. Hint: one of them is easier.)

Submission
----------

#. Submission type: individual submission via iLearn

#. Demo: not required

#. Due time: at the beginning of next lab session

#. Files to submit: **(please do not zip, just upload two files)**

   - A lab report in pdf format
   - open_loop.py (with proper comments)

#. Grading rubric:

   + \+ 50%  Clearly describe your approach and explain your code in lab report.
   + \+ 40%  Your script can drive the robot move along the square trajectory, 
     go back to the origin, **and stop**.
   + \+ 10%  When stop at the origin, the error between the robot and the origin is less than 1.0 meter. 
   + \+ 10%  Bonus points will be given if you can plot the trajectory of your robot with matplotlib.
   + \- 10%  Any missing part in the code or the report where you didn't show a good understanding.
   + \- 15%  Penalty applies for each late day. 

Preview: We will learn how to implement close-loop control next time.


Intro to Turtlebot
------------------

The Turtlebot 2 robot with the Kobuki base is a modular robot, 
on which additional sensors can be placed. 
The robot uses differential drive for locomotion, 
i.e., it has two powered wheels, located symmetrically about its center. 
By commanding the velocity of each of these wheels individually, 
we can obtain the desired linear ``v`` and rotational ``ω`` 
velocities for the robot.

.. image:: pics/frame.PNG
  :width: 60%
  :align: center

To describe the position and orientation of the robot, 
we attach a robot coordinate frame :math:`R` to it. 
The origin of this coordinate frame is centered between its powered wheels. 
The X axis of this frame is pointing forward (along the direction of the linear velocity ``v``),
the Y axis is pointing to the left, and
the Z axis is pointing up.

To track the position and orientation of the robot, 
we generally define a world reference frame :math:`W`, 
in the same plane in which the robot moves. 
With this frame assignment, 
the robot’s position is constrained to the X − Y plane of frame :math:`W`. 
Moreover, any rotation between the robot and the world frames can be expressed 
as a rotation about Z axis. 
Therefore, the position of the robot with respect to the world reference frame will have the form:

.. math::

  P_W = 
  \begin{bmatrix}
  x    \\
  y    \\
  0     
  \end{bmatrix}

while the rotation matrix expressing the orientation of the robot frame 
with respect to :math:`W` will be of the from:

.. math::

  R_{WR} = 
  \begin{bmatrix}
  cos(\phi) & -sin(\phi) & 0  \\
  sin(\phi) & cos(\phi) & 0   \\
  0 & 0 & 1     
  \end{bmatrix}


Sample Code
------------

A sample code is given as the starting point for your implementation. 
Please read carefully the provided code, and understand its functionality. 
Note that the provided code can only drive the robot move straight forward.
Please add the turning part in order to complete the square trajectory.
You can modify any part of the provided code in your final submission.

- Open a new terminal and go to your ``ee144f20`` package. 
  We will start from a new python script.

  .. code-block:: bash

    roscd ee144f20
    mkdir scripts
    cd scripts
    touch open_loop.py
    gedit open_loop.py

- Please copy and paste the following code, then save and close it.

  .. literalinclude:: ../scripts/open_loop.py
    :language: python

- Back to the terminal, you can run it in two ways. 
  One is to feed this script as input to the python program in Linux,
  as shown below.

  .. code-block:: bash

    python open_loop.py

- The other way is to run it as a regular executable in Linux. In this case,
  you need to first grant the execution permission to this Python script.
  This step only need to be run once.

  .. code-block:: bash

    chmod +x open_loop.py

- Now you can see that this file turns to be in green color when you ``ls`` the current directory.
  You may try creating a new empty file and see its color. It should be in white.

  .. code-block:: bash

    ls

- Then you can run it by command

  .. code-block:: bash

    ./open_loop.py

.. note::

  Recall in Lab 1 that you need to first launch your Turtlebot robot in Gazebo
  before sending any commands to it.

  .. code-block:: bash

    roslaunch ee144f20 gazebo.launch


Sample Code Explanations
------------------------

- We will break the above sample code into parts and give explanations. 

- First of all, we need to talk a bit about the *class* concept in Python.
  It is recommended that you write the code using class
  In short, *class* is a set or category of things having some property or 
  attribute in common and differentiated from others by kind, type, or quality. 
  *Object* is one of instances of the class, 
  which can perform the functionalities defined in the class. 
  *Self* represents the instance of the class. 
  By using the *self* keyword we can access the attributes and methods of the class in python.
  ``__init__`` is a reserved method in python classes. 
  It is known as a constructor in object oriented concepts. 
  This method called when an object is created from the class 
  and it allow the class to initialize the attributes of a class.
  For more details, please refer to `this link <https://docs.python.org/2/tutorial/classes.html>`_.

- The first line makes sure your script is executed as a Python script in Linux.
  You need this line if you want to run it as a regular executable in Linux.
  
  .. code-block:: python

    #!/usr/bin/env python

- You need to import rospy in order to use ROS in Python.
  This is the Python library that contains common resources in ROS.

  .. code-block:: python

    import rospy

- This line imports a ``Twist`` class that help us expresses velocity.

  .. code-block:: python

    from geometry_msgs.msg import Twist

- One of the first calls you will likely execute in a rospy program is 
  the call to ``rospy.init_node()``, which initializes the ROS node for the process. 
  You can only have one node in a rospy process, 
  so you can only call ``rospy.init_node()`` once. 
  As part of the ``init_node()`` call, 
  you will pass in the default name of your node. 
  When you run your code, this is the name that your node will appear as online 
  unless it’s overridden by remapping or other arguments. 
  In this case, your node will take on the name ``turtlebot_move``.

  .. code-block:: python

    rospy.init_node("turtlebot_move")

- ``rospy.loginfo(str)`` performs triple-duty: 
  the messages get printed to screen, 
  it gets written to the node’s log file, 
  and it gets written to ``rosout``. 
  ``rosout`` is a handy for debugging: 
  you can pull up messages using rqt console instead of 
  having to find the console window with your node’s output.

  .. code-block:: python

    rospy.loginfo("Press CTRL + C to stop turtlebot")

- You can create a handle to publish messages to a topic 
  using the ``rospy.Publisher`` class. 
  The required arguments to create a ``rospy.Publisher`` are 
  the topic name ``cmd_vel_mux/input/navi``, 
  the message class ``Twist``, 
  and the queue size ``10``.

  .. code-block:: python

    self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)

- TurtleBot will stop if we don’t keep telling it to move. 
  ``rospy`` provides a ``rospy.Rate`` class which allows your loops 
  to run at the rate that you specify. 
  In the below example, the ``Rate`` instance will attempt to keep the loop at 10Hz. 
  With its argument of 10, we should expect to go through the loop 10 times per second.

  .. code-block:: python

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        self.vel_pub.publish(vel)
        rate.sleep()


More ROS Tutorials
------------------

Please read the following pages to have a better understanding of ROS. 
They are helpful for your coding and future labs.

ROS Nodes
~~~~~~~~~

- `Understanding ROS Nodes <http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes>`_.

- `Initialization and Shotdown <http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown>`_.


ROS Topics and Messages
~~~~~~~~~~~~~~~~~~~~~~~

- `Messages <http://wiki.ros.org/Messages>`_.

- `Understanding ROS Topics <http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics>`_.

- `Publishers and Subscribers <http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers>`_.



