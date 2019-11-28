# FMTstar-planner

This repository provides a path-planning ROS service which uses Heuristic FMT*(Forward Marching Tree) planner for 2D ROS occupancy grids in ROS to generate asymptotically optimal paths at a rate much faster than state of the art algorithms like RRT* and PRM*.

### Steps to run it:
1. Clone the Repository

> cd catkin_ws/src 

> git clone https://github.com/YashTrikannad/FMTstar-planner.git 

> catkin_make install

> catkin_make

2. Run the F110 simulator (Make sure you have it cloned on your local machine (Follow the instructions in this repository)- [mLab](https://github.com/mlab-upenn/f110-fall2019-skeletons)) 

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

### How to call the planner and recieve plans

```
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "fmt_star/plan_srv.h"
```

Include the required headers

```
int main(int argc, char **argv)
{
    ros::init(argc, argv, "FMTstar_test_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<fmt_star::plan_srv>("FMTstar_search");
```

Initialize your client node where you want to recieve the path (You may already be doing this where you want to call this.) and add add this service that the client node will be pinging to for recieveing the path messages.

```
    fmt_star::plan_srv srv_message;
    const auto start = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("gt_pose",ros::Duration(2));
    ROS_INFO("Send a 2d Nav Goal");
    const auto goal = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("move_base_simple/goal",ros::Duration(20));
```

Get your start and goal messages which are *geometry_msgs::PoseStamped*

```
    if(!start)
    {
        ROS_ERROR("Unable to Plan. Start not recieved");
    }
    else if(!goal)
    {
        ROS_ERROR("Unable to Plan. Goal not recieved");
    }
    else
    {
        srv_message.request.start_position = *start;
        srv_message.request.end_position = *goal;
    } 
```

Assign your start and goal to the service request

```
    if (client.call(srv_message))
    {
        ROS_INFO("Plan Recieved");
        // Plan available here.
    }
    else
    {
        ROS_ERROR("No Plan Recieved");
    }

    return 0;
}

```

Your Plan will be available in *srv_message.response.path*. You can find the [Test Client Example](https://github.com/YashTrikannad/FMTstar-planner/blob/master/testing/FMTstar_test_client_node.cpp) here

