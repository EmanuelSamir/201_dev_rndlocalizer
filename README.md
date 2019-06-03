# 201_dev_rndlocalizer
This repository is part of a project to implement an algorithm in kobuki (turtlebot 2) in simulation or in real environment
I only test this script in simulation. If you want to test it, this script is a node which works with Gazebo Simulation and with Turtlebot nodes.
Install dependecies and ROS Kinetic for running.
Run following lines of code:
~~~
$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=worlds/willowgarage.world
$ export TURTLEBOT_BASE=create
$ export TURTLEBOT_STACKS=circles
$ export TURTLEBOT_3D_SENSOR=asus_xtion_pro
$ roslaunch turtlebot_gazebo turtlebot_playground.launch
~~~
In another window:
~~~
roslaunch turtlebot_gazebo gmapping_demo.launch
~~~
In another terminal:
~~~
$ roslaunch turtlebot_rviz_launchers view_navigation.launch
~~~
and then run the executable (this script)

FOr more info: Check http://wiki.ros.org/turtlebot_gazebo/Tutorials/indigo/Make%20a%20map%20and%20navigate%20with%20it
