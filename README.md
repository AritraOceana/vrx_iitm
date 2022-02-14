# VRX_iitm
Solutions to Virtual RobotX 2022 challenge

## Wayfinding preliminary solution

This package requires pyproj python library to be installed first. You can install it using pip

```
pip3 install pyproj
```

Now, clone the wayfinding_soln package in your vrx workspace.

To test the package, launch two terminal tabs 

In the first terminal, launch the following .launch file

```
roslaunch wayfinding_soln wayfinding_task.launch
```

In the second terminal, run the solution script

```
rosrun wayfinding_soln wayfinding_task.py
```

You can also check the task details such as **remaining_time** and **score** by running the following command in a new terminal window,

```
rostopic echo /vrx/task/info
```
