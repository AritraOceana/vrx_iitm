# VRX_iitm
Solutions to Virtual RobotX 2022 challenge
<<<<<<< HEAD
=======

## Station keeping preliminary solution

This package requires pyproj python library to be installed first. You can install it using pip

```
pip3 install pyproj
```

Now, clone the station_keeping package in your vrx workspace.

To test the package, launch two terminal tabs 

In the first terminal, you will launch the station keeping world that you want to test:

```
roslaunch vrx_gazebo station_keeping.launch
```
or any custom station keeping world, for ex. :
```
DIFFICULTY=0
roslaunch vrx_gazebo vrx.launch verbose:=true \
	        paused:=false \
	        wamv_locked:=true \
	        world:=${HOME}/vrx_ws/src/vrx/vrx_gazebo/worlds/2022_practice/stationkeeping${DIFFICULTY}.world
```
where you will want to change the value of the DIFFICULTY variable to change the difficulty level of the world you want to test. You can choose DIFFICULTY from this set [0, 1, 2].

In the second terminal, run the solution script

```
roslaunch station_keeping_soln station_keeping.launch
```

You can also check the task details such as **remaining_time** and **score** by running the following command in a new terminal window,

```
rostopic echo /vrx/task/info
```
>>>>>>> station_keeping
