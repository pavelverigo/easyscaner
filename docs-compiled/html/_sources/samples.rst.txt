Samples
########

We have created C++ and Python samples. You can go through the samples below.

Python samples
===============

``getinfo.py``
-------------------
Show basic functionality of Python bindings

.. literalinclude:: ../../samples/python/getinfo.py
	:language: python


``change_lane.py``
-------------------
Functionality of changing lane

.. literalinclude:: ../../samples/python/change_lane.py
	:language: python


``decelerate.py``
-------------------
Get vehicle speeds, and command acceleration profile

.. literalinclude:: ../../samples/python/decelerate.py
	:language: python


``display_message.py``
-------------------
Display text on screen

.. literalinclude:: ../../samples/python/display_message.py
	:language: python


``IDM+.py``
-------------------
IDM+ model

.. literalinclude:: ../../samples/python/IDM+.py
	:language: python


C++ samples
============

``ChangeLane.cpp``
-------------------
Ask all vehicles to move a lane to the left on the start, and move to the right 5 ticks later.

.. literalinclude:: ../../samples/EasyScaner/ChangeLane.cpp
	:language: cpp

``Decelerate.cpp``
-------------------
All vehicles accelerate at the start. When they reach a speed of 20 m/s, they will gradually decelerate.

.. literalinclude:: ../../samples/EasyScaner/Decelerate.cpp
	:language: cpp

``HelloWorld.cpp``
-------------------
Logs a lot of data.

.. literalinclude:: ../../samples/EasyScaner/HelloWorld.cpp
	:language: cpp

``IDM+.cpp``
-------------------
Implements the IDM+ model.

.. literalinclude:: ../../samples/EasyScaner/IDM+.cpp
	:language: cpp
