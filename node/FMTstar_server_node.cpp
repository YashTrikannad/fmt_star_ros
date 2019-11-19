#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "fmt_star/planner.h"
#include "fmt_star/plan_srv.h"

bool get_plan(fmt_star::plan_srv::Request& request, fmt_star::plan_srv::Response& response)
{
    nav_msgs::OccupancyGrid input_map_  = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map",ros::Duration(2)));
    fmt_star::Planner planner(input_map_);
    const auto plan = planner.get_plan(request.start_index, request.end_index);
    if(plan.empty())
    {
        return false;
    }
    for(const auto& node: plan)
    {
        response.path.emplace_back(node);
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FMTstar_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("FMTstar_search", get_plan);
    ROS_INFO("Global Planner Service Ready.");
    ros::spin();
    return 0;
}