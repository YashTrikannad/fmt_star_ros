#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "fmt_star/planner.h"
#include "fmt_star/plan_srv.h"

bool get_plan(fmt_star::plan_srv::Request& request, fmt_star::plan_srv::Response& response)
{
    nav_msgs::OccupancyGrid input_map_  = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map",ros::Duration(2)));
    fmt_star::Planner planner(input_map_, 10000, 1.0);
    const auto plan = planner.get_plan(request.start_index, request.end_index);
    const auto sampled_nodes = planner.get_sampled_nodes();

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 100);

    visualization_msgs::MarkerArray viz_msg;

    for(size_t i=0; i<sampled_nodes.size(); i++)
    {
        visualization_msgs::Marker point;
        std::cout << "i: " << i << std::endl;
        point.header.frame_id = "/other_base_link";
        point.header.stamp = ros::Time::now();
        point.ns = "point_123";
        point.action =visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.id = i;
        point.type = visualization_msgs::Marker::SPHERE;
        point.scale.x = 0.2;
        point.scale.y = 0.2;
        point.scale.z = 0.2;
        point.color.r = 1.0f;
        point.color.g = 0.0f;
        point.color.a = 1.0;
//        point.pose.position.x = get_xy_from_row_major()[i];
//        point.pose.position.y = y_poses_ego_vehicle[i];
        point.lifetime = ros::Duration(10);
        viz_msg.markers.push_back(std::move(point));
    }
//    pub_markers.publish(viz_msg);

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