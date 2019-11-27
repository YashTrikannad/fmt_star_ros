# FMTstar-planner

This repository is being developed for doing path-planning using a Heuristic FMT*(Forward Marching Tree) planner for occupancy grids in ROS.

Steps to run it:
1. Clone the Repository

> cd catkin_ws/src 

> git clone https://github.com/YashTrikannad/FMTstar-planner.git 

> catkin_make install

> catkin_make

2. Run the F110 simulator (Make sure you have it cloned on your local machine (Follow the instructions in this repository)- https://github.com/mlab-upenn/f110-fall2019-skeletons)

> cd catkin_ws

> source devel/setup.bash

> roslaunch racecar_simulator simulator.launch

3. Open a new terminal. Run the Server Node

> cd catkin_ws

> source devel/setup.bash

> roslaunch fmt_star fmt_star_server.launch

4. Open a new terminal. Run the Client Node to query the planner

> cd catkin_ws

> source devel/setup.bash

> rosrun fmt_star FMTstar_test_client_node


Configuration Parameters can be changed in the config/config.yaml file to get the desired behavior and efficiency
