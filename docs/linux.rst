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

  .. code-block:: bash

    roscd ee144

- You will see that it becomes

  .. code-block:: bash

    roscd ee144f20

- This means that Linux can identify an **unique** directory (or filename, argument, etc.) 
  by just seeing the first five characters. 

- You can try the following command again and press ``Tab`` key once.

  .. code-block:: bash

    roscd e

- Nothing happens, right? Because in this case, Linux cannot **uniquely** identify 
  which command you want to enter, and hence cannot help to complete it.
  There are many commands (ROS packages actually) starting with "e" in your system.
  
- In this case, you can try **double** pressing ``Tab`` key, 
  then you can see all the possible options starting with "e" in your system.


Terminal and Shell
------------------

- You can see the relationship between terminal and shell by command

  .. code-block:: bash

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

  .. code-block:: bash

    cd ~/catkin_ws/devel
    ls


Terminal and File Manager
-------------------------

Similarly, there are `many types of file managers 
<https://www.ubuntupit.com/linux-file-manager-reviewed-for-linux-users/>`_ available in Linux. 
Ubuntu 16 has ``nautilus`` as the default file manager. 

We can go back and forth between terminal and file manager at any working directory.

- Suppose that you are now at your home directory. 
  (``cd`` without any argument will take you to your home directory under your account.)

  .. code-block:: bash

    cd

- You can open file manager from terminal by command

  .. code-block:: bash

    nautilus .

- where ``nautilus`` is the name of the program you are trying to run, 
  and ``.`` is the argument passing into ``nautilus`` that represents current directory.

- You can also open file manager at any other working directory. 
  For example, go to ``ee144f20`` package and open file manager from this directory.

  .. code-block:: bash

    roscd ee144f20
    nautilus .

- On the other hand, at any level of file manager, you can open a new termimal by just 
  a right click and select "Open in Terminal".

- Note that if a program is currently running in Terminal, 
  you will lose the ability to interact with it by typing new commands.
  (You can tell this by seeing if you have ``username@hostname:~$`` prompted in your terminal,
  where ``~`` can be other working directory.)
  If you want to reuse the same termimal for typing new commands, you can do

  .. code-block:: bash

    roscd ee144f20
    nautilus . &

- where ``&`` can combine two commands. In this case, no new command is given, 
  and hence it takes you back to your terminal, and have the previous command run in backend.

- You can also use this trick when you open ``gedit`` editor or other graphic tools like ``rqt_graph``.

  .. code-block:: bash

    roscd ee144f20/launch
    gedit gazebo.launch &


Shortcuts
---------

- To open a new terminal, press key combination ``Ctrl + Alt + T``, where T stands for Terminal.

- To copy and paste a file, you typically use ``Ctrl + C`` and ``Ctrl + V`` anywhere else. 
  In terminals, the corresponding commands are ``Ctrl + Shift + C`` and ``Ctrl + Shift + V``.

- To terminate a program in terminal, press key combination ``Ctrl + C``.

- To show hidden files in your file manager, press key combination ``Ctrl + H``, where H stands for Hidden.

- To show hidden files in terminal, use command ``ls -a``, where ``a`` stands for ``all``.
  This command will list all files including hidden ones. 
  (In Linux, files start with ``.``, or files only have suffixes, are hidden files.)

- In terminal, you can use up ``↑`` and down ``↓`` arrow keys to go through your command history.

- ``.`` is your current working directory, ``..`` is your parent working directory,
  and ``~`` is your default home directory under your account.
  For example, ``cd ..`` can take you back to your parent directory, 
  and ``cd .`` will keep you staying at current directory (nothing changed).

- To zoom in and zoom out in termimal, 
  use key combination ``Ctrl + Shift + "+"`` and ``Ctrl + "-"``, respectively.

- To reset your robot to its initial pose in Gazebo, use key combination "Ctrl + R", where R stands for reset.


Environment Variables
---------------------

This is a bit advanced concept.

- You can check your ROS-related environment variables by command

  .. code-block:: bash

    env | grep ROS
