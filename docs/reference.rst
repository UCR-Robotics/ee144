Reference Materials
===================

Atom Editor
-----------

``gedit`` is the default editor available in Ubuntu OS,
which sometimes is hard to use.
Here I personally recommend a lightweight editor called `Atom <https://atom.io/>`_.
It has a couple good features, such as auto-completion, file system browser,
Tab key detection, and so on. 

- To install Atom, first add the official repository to the package list.

.. code:: bash

    wget -qO - https://packagecloud.io/AtomEditor/atom/gpgkey | sudo apt-key add -
    sudo sh -c 'echo "deb [arch=amd64] https://packagecloud.io/AtomEditor/atom/any/ any main" > /etc/apt/sources.list.d/atom.list'
    
- Then update your apt package list and install Atom.

.. code:: bash
    
    sudo apt-get update
    sudo apt-get install atom

- After installation, you can open Atom by just typing atom in your terminal.

.. code:: bash
    
    atom

- You can right click the Atom icon on taskbar and choose "Lock to Launcher",
  such that you can run it by just click the icon on taskbar next time.

- You can customize some settings in Atom. Go to the menubar of Atom.
  Open settings by click on "edit" first, and then "preferences".
  Go to Editor, then you can change font size and tab length 
  (how many spaces you want to have when you press a Tab key).
  This is really useful especially when working with Python,
  since you have to indent each line of the block by the same whitespace,
  in order to indicate a block of code in Python.


Terminals
---------

Coming soon.



