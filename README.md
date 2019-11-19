# FMTstar-planner

This repository is being developed for doing path-planning using FMTstar-planner for occupancy grids in ROS.

Steps to run it:
1. Clone the Repository

git clone https://github.com/YashTrikannad/FMTstar-planner.git

2. Run the F110 simulator (Make sure you have it cloned on your local machine- https://github.com/mlab-upenn/f110-fall2019-skeletons/tree/master/racecar_simulator)

roslaunch racecar_simulator simulator.launch

3. Run the Server Node

rosrun fmt_star FMTstar_server_node

4. Run the Client Node

rosrun fmt_star FMTstar_test_client_node

