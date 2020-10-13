Linux Tutorials
===============

In the following we will show some basic commands, shortcuts, and tricks for playing with Linux.

Tab Key Auto-completion
-----------------------

- The most commonly used one is the ``Tab`` key. 
  This auto-completion feature is universal in all Linux systems.
  If you cannot auto-complete the command you are currently typing, 
  **this means that your command is wrong** and Linux is not able to recognize it.
  Try entering the following command in your terminal and press ``Tab`` key on the keyboard.

  .. code-block:: bash

    roscd ee144

- You will see that it becomes

  .. code-block:: bash

    roscd ee144f20

- This means that Linux can identify an **unique** name of the directory
  by just seeing the first five characters. 

- You can try the following command again and press ``Tab`` key once.

  .. code-block:: bash

    roscd e

- Nothing happens, right? Because in this case, Linux cannot **uniquely** identify 
  which command you want to enter, and hence cannot help complete it.
  There are many ROS packages starting with "e" in your system.
  
- In this case, you can try **double** pressing the ``Tab`` key, 
  then you can see all the possible options starting with "e" in your system.


Terminal and Shell
------------------

- You may have already seen some files ended up with a suffix ``.bash`` or ``.sh``.
  Though Linux does not count on suffix to determine the file type, but this indicates
  that they are Shell scripts. 
  Similar to Python scripts, Shell scripts are those that can be executed by a shell program in the terminal.
  In general, its syntax consists of most of the commands that you can directly type in the terminals.

- One example to see the relationship between terminal and shell. Try command

  .. code-block:: bash

    pstree | grep bash

- This will show you current running programs related to the keyword ``bash``.
  You can see that ``bash`` is running after the ``gnome-terminal``, 
  and the ``grep`` command is running after ``bash``.

- In general, there are many types of terminals and shells available in Linux.
  Specifically, in Ubuntu 16.04, it has ``gnome-terminal`` as the default terminal 
  and ``bash`` as the default shell. 
  
- Roughly speaking, you can think of terminal as the frontend GUI and shell as the actual program 
  that executes your commands in the backend (GUI is also a program though).
  You may Google keywords "gnome-terminal" or "sh, zsh, bash" for more information.

- If you go to the ``devel`` folder in the ROS workspace, 
  you can see that the ROS ``setup`` file is available for three kinds of shells: sh, zsh, and bash.

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
  (``cd`` command by itself without any argument will take you to the home directory.)

  .. code-block:: bash

    cd

- You can open file manager from terminal by command

  .. code-block:: bash

    nautilus .

- where ``nautilus`` is the name of the program you are trying to run, 
  and ``.`` is the argument passing into ``nautilus`` that represents the current directory.

- You can also open a file manager at any other working directory. 
  For example, go to the ``ee144f20`` package and open a file manager from this directory.

  .. code-block:: bash

    roscd ee144f20
    nautilus .

- On the other hand, at any level of file manager, you can open a new terminal by just 
  a right click and select "Open in Terminal".

- Note that if a program is currently running in Terminal, 
  you will lose the ability to interact with it by typing new commands.
  (You can tell this by seeing if you have ``username@hostname:~$`` prompted in your terminal,
  where ``~`` can be another working directory.)
  If you want to reuse the same terminal for new commands, you can do

  .. code-block:: bash

    roscd ee144f20
    nautilus . &

- where ``&`` can combine two commands. In this case, no new command is given, 
  and hence it takes you back to your terminal and has the previous command run in the backend.

- You can also use this trick when you open ``gedit`` editor or other graphic tools like ``rqt_graph``.
  This can free your current terminal while opening other software.

  .. code-block:: bash

    roscd ee144f20/launch
    gedit gazebo.launch &


Shortcuts
---------

- To open a new terminal, press key combination ``Ctrl + Alt + T``, where T stands for Terminal.

- To copy and paste a file, you can use ``Ctrl + Shift + C`` and ``Ctrl + Shift + V`` in terminals,
  and use ``Ctrl + C`` and ``Ctrl + V`` anywhere else.

- To terminate a program in the terminal, press key combination ``Ctrl + C``.

- To show hidden files in your file manager, press key combination ``Ctrl + H``, where H stands for Hidden.

- To show hidden files in the terminal, use command ``ls -a``, where ``a`` stands for ``all``.
  This command will list all files including hidden ones. 
  In Linux, files start with ``.`` (i.e. only have suffixes) are hidden files.

- In terminal, you can use up ``↑`` and down ``↓`` arrow keys to go through your command history.

- ``.`` stands for the current working directory; ``..`` stands for the parent working directory;
  and ``~`` stands for the default home directory under your account.
  For example, ``cd ..`` can take you one-level back to the parent directory, 
  and ``cd .`` will keep you staying at the current directory (does nothing).

- To zoom in and zoom out in the terminal, 
  use key combination ``Ctrl + Shift + "+"`` and ``Ctrl + "-"``, respectively.

- To reset your robot to its initial pose in Gazebo, use key combination "Ctrl + R", where R stands for reset.


Environment Variables
---------------------

This is an advanced concept that might be out of the scope of this course. List it here for your information.

- You can check ROS-related environment variables by command

  .. code-block:: bash

    env | grep ROS

