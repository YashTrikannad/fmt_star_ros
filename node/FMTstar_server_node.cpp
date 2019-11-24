#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "fmt_star/planner.h"
#include "fmt_star/plan_srv.h"


bool get_plan(fmt_star::plan_srv::Request& request, fmt_star::plan_srv::Response& response)
{
    ros::NodeHandle nh;
    int n_samples;
    double near_radius;
    std::array<double, 4> rectangular_sampling_limits{};
    nh.getParam("n_samples", n_samples);
    nh.getParam("near_radius", near_radius);
    nh.getParam("x_min", rectangular_sampling_limits[0]);
    nh.getParam("x_max", rectangular_sampling_limits[1]);
    nh.getParam("y_min", rectangular_sampling_limits[2]);
    nh.getParam("y_max", rectangular_sampling_limits[3]);

    nav_msgs::OccupancyGrid input_map_  = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map",ros::Duration(2)));

    fmt_star::Planner planner(input_map_, n_samples, near_radius, rectangular_sampling_limits);
    const auto plan = planner.get_plan({request.start_position[0],request.start_position[1]} ,
                                       {request.end_position[0],request.end_position[1]});

    const auto sampled_nodes = planner.get_sampled_nodes();
    for(const auto& node:sampled_nodes)
    {
        response.x_nodes.emplace_back(node[0]);
        response.y_nodes.emplace_back(node[1]);
    }

    if(plan.empty())
    {
        ROS_INFO("Plan not found");
        return true;
    }
    for(const auto& node: plan)
    {
        response.path_x.emplace_back(node[0]);
        response.path_y.emplace_back(node[0]);
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