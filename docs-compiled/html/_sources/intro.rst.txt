Introduction
#############


About
======
The software is meant to be used with the SCANeR software developed by AVSimulation. It allows you to have the autonomous vehicles in SCANeR adhere to more realistic and complex traffic models rather than the default behaviour that AVSimulation has implemented.
The client deemed this necessary, as the default implementation for the autonomous vehicles is simplistic and unrealistic. This application should fix that problem when used correctly, which allows our client to perform more realistic experiments.

The software was developed by a team of TU Delft students as part of their Computer Science and Engineering studies.
The team members are Ties Bloemen, Oisín Hageman, Pavel Verigo, Medard Svilvásy, and Brent Meeusen.

Setup
======
This section describes how to install the necessary programs and tools to use the application, and to continue development.
It is written for a Windows machine with Visual Studio as IDE.
Other setups might work, but are not tested and are likely to differ in installation.

Visual Studio
--------------
1. Download the latest ``Visual Studio`` from `this page <https://visualstudio.microsoft.com/>`_.
2. Open ``Visual Studio Installer``.
3. Click ``Modify`` in the ``Visual Studio [version]`` section. In this guide, ``[version]`` is ``Community 2019``.
4. Select ``Desktop development with C++``. Select your install process, then click ``Modify``. Wait for the tools to be downloaded and installed.
5. Open your ``Visual Studio`` version. Choose ``Open a local folder`` and select the project.

To build executables, in the top bar click ``Build``,then choose ``Build all`` to build the changed files or ``Rebuild all`` to rebuild every executable.

Use executables
----------------
After building the executables, you should add them as a process in SCANeR.

1. In SCANeR, click "Configuration" and "Configuration Manager...", which will open a Configuration Manager window.
2. Click "Add Process" in the bottom left.
3. In the Process Parameters section, select the correct executable. If you wish, you can change the process name afterwards as well. Click "Add", then "Close".
4. Click "Apply", then "OK" in the Configuration Manager window.

When starting a simulation, make sure that your newly added process is also running.
If it is, congratulations! The simulation now uses your process.

Use Python
-----------
1. Use the Command Prompt to start your Python script. How this is done is dependent on how Python is installed on your system.
2. Open SCANeR and start the simulation.

Your Python snippet should now influence the simulation.

Build documentation
--------------------
Our documentation is generated through ``Doxygen`` and ``Sphinx``.
Here, we will explain how to generate a new documentation.
This is only necessary if the code and the comments were changed. You will not need this when using the application.

.. note::
   Follow the guides below in the order they are presented in.
   Some of them require the guides above to be completed successfully.


.. _Download and setup Doxygen:

Download and setup Doxygen
^^^^^^^^^^^^^^^^^^^^^^^^^^^
1. Download ``Doxygen`` `here <https://doxygen.nl/download.html>`__.
2. Add the executable to the ``Path`` environment variable.
	1. Go to ``Settings > System > About``.
	2. Scroll down, and click ``Advanced system settings``.
	3. Click ``Environment variables...``.
	4. Select ``Path`` in either ``User variables for [user]`` or ``System variables``.
	5. Click ``Edit``. In the new window, click ``New``.
	6. Paste the path to the folder in which ``doxygen.exe`` is located. If not changed manually, this should be ``C:\Program Files\doxygen\bin``. Note that ``\doxygen.exe`` is omitted.
	7. Click ``OK`` on all open windows.
3. Check if ``Doxygen`` is setup correctly.
	1. Open the Command Prompt.
	2. Type ``where doxygen``. If setup correctly, it should show the path to ``doxygen.exe``.
	3. If it says ``INFO: Could not find files for the given pattern(s).``, repeat steps 2 and 3. Make sure you copy the path to the folder where ``doxygen.exe`` is located.
	4. Close the Command Prompt.

Download and setup Python
^^^^^^^^^^^^^^^^^^^^^^^^^^
1. Download ``Python`` `here <https://www.python.org/downloads/>`__. The tool is tested with ``Python 3.9`` and ``Python 3.10``, so it is strongly recommended to use one of these.
2. In the installer, make sure you check the box ``Add Python [version] to PATH``. Unless you want a specific installation, choose ``Install now``.
3. Check if ``Python`` is setup correctly.
	1. Open the Command Prompt.
	2. Type ``where python``. If setup correctly, it should show the path to ``python.exe``.
	3. If it says ``INFO: Could not find files for the given pattern(s).``, try setting the ``Path`` variable manually (step 2 of :ref:`Download and setup Doxygen`).
	4. Close the Command Prompt.

Download and setup Python modules
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1. Open the Command Prompt.
2. Run ``pip install sphinx``. Follow the instructions of the terminal.
3. Run ``pip install breathe``. Follow the instructions of the terminal.
4. Run ``pip install sphinx_rtd_theme``. Follow the instructions of the terminal.

Generate documentation
^^^^^^^^^^^^^^^^^^^^^^^
1. Open the Command Prompt.
2. Navigate to the source code, then ``docs/``.
3. Run ``doxygen``. When successful, it should end with ``finished...``.
4. Run ``sphinx-build -b html source build``. When successful, it should end with ``The HTML pages are in build.``.

Support
========
Tell people where they can go to for help. It can be any combination of an issue tracker, a chat room, an email address, etc.


