FAQ / Troubleshooting
###########################

**TODO** Split Troubleshooting from FAQ

Difference between ``vehicles`` and ``road_vehicles`` in API.
=============================================================

SCANER considers cyclist, pedestrian and some static objects as vehicles, therefore returning them all the time is uncomfortable.
So instead of calling ``api.get_vehicles()`` and checking types, we provide ``apiget_road_vehicles()`` which will not consider pedestians and others as vehicles.

ModuleNotFoundError: No module named 'pyscaner'
===========================================================

This means that Python package installation failed, try to ``pip list``.

If pyscaner is not in the list probably our installation script failed or installed
package to different python version that installed on your computer. 

You can use ``py -0p`` for listing python versions installed on your computer. Try to chagne between them (how?), and use ``pip list``.

**#TODO** This section should include better logic for debugging


CMake STUDIO_DIR environment varible is not found.
===========================================================

Our compilation script did not found AVSimulation folder on the computer,
this did not happend if installation was done by default of SCAnER.

We will need to change Enviromental varibles on the system, add STUDIO_PATH varible pointing to installation folder
by default this should be ``C:\AVSimulation`` (note you should **NOT** put path for SCANeR folder ``C:\AVSimulation\SCANeR1.9``).

**#TODO** Add pictures for fixing this issue
