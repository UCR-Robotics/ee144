Lab 0: Preparation
==================

General Information
-------------------

Here are some general ideas that can help you get prepared for this lab section.

When working with robots, we need skills in Linux, ROS, Python, Git/GitHub,
and Virtual Machine (VM).

1. The best way to learn **Linux** is to spend time playing with it.
   It's just like the first time you had your Windows/Mac computer.
   Additionally, you may follow some tutorials online
   and try those commonly used commands in terminals. 
   For example, it is recommended to go over chapter 1-3 of 
   `this tutorial <http://swcarpentry.github.io/shell-novice/>`_.

2. `Robot Operating System (ROS) <https://www.ros.org/>`_
   is a Linux software library specifically designed for programming on robots.
   It has some features of OS, but is not actually an independent OS.
   With ROS, people do not need to worry about low-level communications and
   keep reinventing the wheel.
   On top of ROS, developers all over the world can work on their own software
   packages, and contribute to ROS community.
   These packages are similar to those libraries that we "import" in Python.

3. A good way to learn **ROS** is to learn from `ROS wiki <http://wiki.ros.org/ROS/Tutorials>`_,
   which provides official tutorials.
   In addition, there is a reference book *A Gentle Introduction to ROS*
   by Jason M. O’Kane (`free online <https://www.cse.sc.edu/~jokane/agitr/>`_), 
   which introduces useful design ideas behind ROS.
   Note that examples in this book and some tutorials on ROS wiki are written in C++.
   Please focus on high-level design ideas and `rospy <http://wiki.ros.org/rospy_tutorials>`_
   library only (not roscpp), since we will use Python (instead of C++) in this class.

4. For **Python**, basically you need to have a rough idea about data structures,
   operators, flow control, etc. There are also many good tutorials online.
   For example, the tutorial on `W3Schools <https://www.w3schools.com/python/>`_.
   Going through the first 20 sections (until Python Functions) would be sufficient for this class.

5. **Git** is a version control tool and `GitHub <https://github.com/>`_
   is a website (or company) that offers Git-based version control service.
   It's good to learn Git in the sense that you can better manage your code.
   With Git, you can see all your change history, and have backups of each version.
   Many ROS packages that we are going to use in this course, are hosted on GitHub.
   However, it's not strictly required in this class. 
   Going through chapter 1-5 of `this tutorial <http://swcarpentry.github.io/git-novice/>`_ 
   might be helpful.

6. For **Virtual Machine**, there are mainly two kinds of software available online.
   One is `VMware <https://www.vmware.com/>`_ and the other is
   `VirtualBox <https://www.virtualbox.org/>`_.
   The former has better utilization of GPU and hence supports better graphics, 
   but it is not free of charge.
   The good news is that in recent years VMware has released a free "Player" version for
   individual users, which we will discuss later.
   On the other hand, VirtualBox is totally open source (free) for all platforms (Windows, Mac, Linux).
   However, it does not perform well in heavy simulation tasks in Gazebo.
   (Gazebo is a simulator that we are going to use throughout the course, together with ROS.)


Please familiarize yourself with the above concepts/tools, if they are new to you.

In the following, we will go through some basic steps to get our development environment ready.

Virtual Machine
---------------

- If you have a Linux laptop, or you can dual boot with Linux operating system,
  that's great! This is the best way to work on robots.
  (Please be careful about dual boot, since you have to take potential risks.
  We recommend using VMware instead.)

- If you have a Windows laptop, please go for
  `VMware Workstation Player <https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html>`_.
  Note that this version 15.5 should be free, as I see version 16 is not free of charge any more.

- If you have a Mac laptop, please go to `VMware Fusion <https://www.vmware.com/products/fusion.html>`_
  webpage, register under "Get a Free Personal Use License" tab and download **VMware Fusion Player**
  using free personal license.


Install Linux
-------------

Once you have your VMware installed, let's create a new VM and install Ubuntu 16.04.

- Download Ubuntu 16.04 disc image from
  `official website <http://releases.ubuntu.com/16.04/>`_ (64-bit PC Desktop).

- In VMware, create a new VM.

  + Typical configuration
  + Choose the disc image you download
  + Enter some information about this VM
  + Again, enter name
  + Please allocate at least 30GB (preferred 50GB or more)
  + Store virtual disk as a single file
  + Customize Hardware: Please allocate more memory and CPU processors for better performance
  + Finish

- Great. Now you have a (virtual) Linux computer. Take your time and play with it!

Note that the disk size 30GB/50GB will not be allocated instantly,
but will grow gradually as you add more stuff.


Install ROS
-----------

Once you are familiar with Linux, you can start installing ROS.
In general, we need to follow ROS
`installation tutorial <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_.
Main steps are

- Setup sources.list

  .. code-block:: bash

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

- Setup your keys

  .. code-block:: bash

    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
      
- Update package index

  .. code-block:: bash

    sudo apt-get update

- Install ROS desktop full

  .. code-block:: bash

    sudo apt-get install ros-kinetic-desktop-full

- Environment setup

  .. code-block:: bash

    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

- Install more dependencies

  .. code-block:: bash

    sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

- Initialize rosdep

  .. code-block:: bash

    sudo rosdep init
    rosdep update

Learn from ROS Tutorials
---------------------------

Once you have ROS Kinetic installed, you can follow the tutorials
on `ROS wiki <http://wiki.ros.org/ROS/Tutorials>`_ and
`rospy <http://wiki.ros.org/rospy_tutorials>`_ documentation.

Have fun!
