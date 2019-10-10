Linux Tutorials
===============

Here we will show some basic commands, shortcuts, and tricks when playing with Linux.

Tab Key Auto-completion
-----------------------

- The most commonly used one is the ``Tab`` key. 
  This is the fundamental auto-completion feature in all kinds of Linux terminals.
  If you cannot auto-complete the command you are currently typing, 
  **this means your command is wrong** and Linux cannot recognize it.
  Try entering the following command in your terminal and press ``Tab`` key on your keyboard.

  .. code:: bash

      roscd ee144

- You will see that it becomes

  .. code:: bash

      roscd ee144f19

- This means that Linux can identify an **unique** directory (or filename, argument, etc.) 
  by just seeing the first five characters. 

- You can try the following command again and press ``Tab`` key once.

  .. code:: bash

      roscd e

- Nothing happens, right? Because in this case, Linux cannot **uniquely** identify 
  which command you want to enter, and hence cannot help to complete it.
  There are many commands (ROS packages actually) starting with "e" in your system.
  
- In this case, you can try **double** pressing ``Tab`` key, 
  then you can see all the possible options starting with "e" in your system.


Terminal and Shell
------------------

- You can see the relationship between terminal and shell by command

  .. code:: bash

      pstree | grep bash

- This will show you current running programs (processes actually) related to the keyword ``bash``.
  You can see that bash is running on top of something called ``gnome-terminal``, 
  and ``grep`` command is running within bash.

- Generally speaking, there are many types of terminals and shells available in Linux.
  Specifically, in Ubuntu 16.04, it has ``gnome`` as the default terminal 
  and ``bash`` as the default shell. 
  
- Very roughly speaking, you can think of terminal as the frontend GUI and shell as the actual program 
  that executes your commands in the backend (GUI is also a program though).
  You may Google keywords "gnome-terminal, x-term", or "sh, zsh, bash" for more information.

- If you go to the ``devel`` folder in your ROS workspace, 
  you can see that ROS supports three different kinds of shells: sh, zsh, and bash.
  It has the ``setup`` file available in these three suffixes.

  .. code:: bash

      cd ~/catkin_ws/devel
      ls


Terminal and File Manager
-------------------------

Still, there are `many types of file managers 
<https://www.ubuntupit.com/linux-file-manager-reviewed-for-linux-users/>`_ available in Linux. 
Ubuntu 16 has ``nautilus`` as the default file manager. 

We can go back and forth between terminal and file manager at any working directory.
Coming soon.


Shortcuts
---------

- To open a new terminal, press key combination ``Ctrl + Alt + T``, where T stands for Terminal.

- To show hidden files in your file manager, press key combination ``Ctrl + H``, where H stands for Hidden.

- To show hidden files in terminal, use command ``ls -a``, where "a" stands for "all".
  This command will list all files including hidden ones.


Environment Variables
---------------------

This is a bit advanced concept.

- You can check your ROS-related environment variables by command

  .. code:: bash

      env | grep ROS
