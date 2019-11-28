#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include "fmt_star/planner.h"
#include "fmt_star/plan_srv.h"


class PlannerService
{
public:
    PlannerService(ros::NodeHandle &nh): nh_(nh), rectangular_sampling_limits_({})
    {
        nh_.getParam("n_samples", n_samples_);
        nh_.getParam("near_radius", near_radius_);
        nh_.getParam("n_collision_checks", n_collision_checks_);
        nh_.getParam("obstacle_inflation_radius", obstacle_inflation_radius_);
        nh_.getParam("goal_tolerance", goal_tolerance_);
        nh_.getParam("x_min", rectangular_sampling_limits_[0]);
        nh_.getParam("x_max", rectangular_sampling_limits_[1]);
        nh_.getParam("y_min", rectangular_sampling_limits_[2]);
        nh_.getParam("y_max", rectangular_sampling_limits_[3]);
        nh_.getParam("online", online_);
        nh_.getParam("visualization", visualization_);
        nh_.getParam("hg_ratio", hg_ratio_);

        tree_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("tree_viz", 1000);
        path_pub_ = nh_.advertise<visualization_msgs::Marker>("path_viz", 1000);
    }

    bool get_plan(fmt_star::plan_srv::Request& request, fmt_star::plan_srv::Response& response)
    {
        ROS_INFO("Start and Goal Recieved by the Planner");

        nav_msgs::OccupancyGrid input_map_  = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map",ros::Duration(2)));

        fmt_star::Planner planner(input_map_,
                                  n_samples_,
                                  near_radius_,
                                  n_collision_checks_,
                                  obstacle_inflation_radius_,
                                  goal_tolerance_,
                                  hg_ratio_,
                                  online_,
                                  rectangular_sampling_limits_,
                                  visualization_,
                                  &tree_pub_,
                                  &path_pub_);

        const auto plan = planner.get_plan({request.start_position.pose.position.x,request.start_position.pose.position.y},
                                           {request.end_position.pose.position.x,request.end_position.pose.position.y});


        if(plan.empty())
        {
            ROS_INFO("Plan not found");
            return false;
        }
        ROS_INFO("Sending Plan");

        nav_msgs::Path path;
        path.header.frame_id = "/map";
        path.header.stamp = ros::Time::now();
        for(const auto& node: plan)
        {
            geometry_msgs::PoseStamped path_node;
            path_node.pose.position.x = node[0];
            path_node.pose.position.y = node[1];
            path_node.pose.orientation.w = 1;
            path.poses.emplace_back(path_node);
        }
        response.path = path;
        return true;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Publisher tree_pub_;

    int n_samples_;
    double near_radius_;
    int n_collision_checks_;
    int obstacle_inflation_radius_;
    double goal_tolerance_;
    bool online_;
    double hg_ratio_;
    std::array<double, 4> rectangular_sampling_limits_;

    bool visualization_;
    visualization_msgs::MarkerArray viz_msg;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "FMTstar_server");
    ros::NodeHandle n;
    PlannerService planner(n);
    ros::ServiceServer service = n.advertiseService("FMTstar_search", &PlannerService::get_plan, &planner);
    ROS_INFO("Global Planner Service Ready.");
    ros::spin();
    return 0;
}