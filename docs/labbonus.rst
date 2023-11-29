Lab Bonus
===========

Overview
--------
This is a bonus programming assignment following the topic of Forward Kinematics in Lab 4.
The score of this assignment (100 bonus points) will be added as a 10% bonus. 
The problem is to derive and code the forward kinematics of the arm shown in Fig 4.18
in the textbook using the D-H formulation (50 points) and the PoE formula (50 points).  

Please follow the instructions as listed below.

- Please use exactly the same file name and function name for submission.

- Note that this time you need to return **the final transformation matrix T**, 
  as opposed to position only in Lab 4.

- There will be 5 test cases for each approach on autograder. 
  All scripts will be double checked. 

- It is required to use the **PoE** and the **D-H parameters** to solve the problem.
  In other words, PoE approach must be used in the ``forward_kinematics_poe.py`` script,
  and D-H approach must be used in the ``forward_kinematics_dh.py`` script.
  Otherwise penalty will apply and points will be deducted.

- Since this is a bonus, and to be fair with everyone, 
  there will be no late submission option available, 
  and we will not offer the kind of feedback we gave for Lab 4.

- No need to submit the lab report, as this is not a regular lab assignment. 


Submission
----------

#. Submission: individual submission via Gradescope

#. Due time: 11:59pm, Dec 11, Monday

#. Files to submit:

   - forward_kinematics_poe.py
   - forward_kinematics_dh.py

#. Grading rubric:

   + \+ 50 pts   Product of Exponentials
   + \+ 50 pts   D-H Parameters


Sample Code
-----------

- Two sample Python scripts are provided as follows. 

  .. literalinclude:: ../scripts/forward_kinematics_poe.py
    :language: python

  .. literalinclude:: ../scripts/forward_kinematics_dh.py
    :language: python


Specification
-------------

Figure 4.18 in the textbook is shown below. 

.. image:: pics/bonus_fig_4.18.png
