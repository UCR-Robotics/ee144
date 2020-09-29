Lab 7: Motion Planning
======================

Overview
--------

In this lab, we will work on the second stage of motion planning, 
which is the A* graph search algorithm. 
With this algorithm, you can find an optimal path given a (square) grid map.
We only consider exact cell decomposition 
(no multi-resolution grids, no mixed cells, 
if any part of the grid is occupied, the entire grid is marked as obstacle).
You can pick Manhattan distance or Euclidean distance as the Heuristic function.
No diagonal movement is allowed (4-connected graph). 

#. You need to first decompose your environment into (square) grid map,
   marked with start point, end point, and obstacles.
#. Second, transform the grid map into a graph with nodes and weights assigned.
#. Finally, run A* algorithm to search an optimal path and return with all the waypoints on the path.

Please refer to your lecture slides for more details. 
An example of pseudocode is provided in the end of this page.

Preview: We will play with sensors and work on ROS navigation stack next time.


Submission
----------

#. Submission: group submission (from one of you) via iLearn, 
   due by the end of the day of next lab

#. Demo: required, due by the end of next lab

#. Files to submit: **(please do not zip, just upload all files)**

   - lab7_report_01.pdf (replace 01 with your team number, **use number 01-18**)
   - motion_planning.py
  
#. Grading rubric:

   - \+ 50%  Clearly describe your approach and explain your code in lab report.
   - \+ 30%  Decompose the environment and implement A* algorithm for motion planning.
   - \+ 20%  Apply A* algorithm on the robot and be able to move from the 
     start point to the end point without colliding with (given) obstacles.
   - \- 15%  Penalty applies for each late day. 


Code Snippets
-------------

- Open a new terminal and go to your ``ee144f20`` package. 
  We will create a new python script.

  .. code-block:: bash

    roscd ee144f20/scripts
    touch motion_planning.py
    gedit motion_planning.py

- Please copy and paste the code from previous lab, 
  then you may want to add the following modules.

  .. code-block:: python

    class Turtlebot():
        def __init__(self):
            rospy.init_node("turtlebot_move")
            # some other code ...


        def get_path_from_A_star(self):
            start_point = [0, 0]
            end_point = [5, 1]
            obstacles = [[2, -1], [2, 0], [2, 1], ...]
            # you may want to add more boundary grids to obstacles

            open_list = []
            close_list = []
            optimal_path = []

            return optimal_path


        def run(self):
            # get waypoints from A star algorithm
            waypoints = self.get_path_from_A_star()
            for point in waypoints:
                self.move_to_point(point)
            self.stop()
            rospy.loginfo("Action done.")


        # some other functions ...


- The environment setup is the following, where the green grid is the start position,
  the red grid is the end position, and the grey grids are obstacles. 
  If applying to the testing area in our lab, the grid size should be 0.5m.

  .. image:: pics/map.jpg
    :width: 60%
    :align: center
  
- The final result should be similar to the following.

  .. image:: pics/result.jpg
    :width: 60%
    :align: center

- Remember that you can test your code in Gazebo first and then apply to real robot.


A* Pseudocode
-------------

- You may refer to the pseudocode shown below.

.. image:: pics/pseudocode.jpg
  :width: 80%
  :align: center

