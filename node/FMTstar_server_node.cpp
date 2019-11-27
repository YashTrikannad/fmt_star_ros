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
        nh_.getParam("visualization", visualization_);

        node_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("sampled_nodes", 1000);
    }

    bool get_plan(fmt_star::plan_srv::Request& request, fmt_star::plan_srv::Response& response)
    {
        nav_msgs::OccupancyGrid input_map_  = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map",ros::Duration(2)));

        fmt_star::Planner planner(input_map_,
                                  n_samples_,
                                  near_radius_,
                                  n_collision_checks_,
                                  obstacle_inflation_radius_,
                                  goal_tolerance_,
                                  rectangular_sampling_limits_);

        const auto plan = planner.get_plan({request.start_position[0],request.start_position[1]} ,
                                           {request.end_position[0],request.end_position[1]});

        const auto sampled_nodes = planner.get_sampled_nodes();
        if(!sampled_nodes.empty() && visualization_)
        {
            visualize_sample_nodes(sampled_nodes);
        }

        if(plan.empty())
        {
            ROS_INFO("Plan not found");
            return false;
        }
        ROS_INFO("Sending Plan");
        for(const auto& node: plan)
        {
            response.path_x.emplace_back(node[0]);
            response.path_y.emplace_back(node[0]);
        }
        return true;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher node_pub_;

    int n_samples_;
    double near_radius_;
    int n_collision_checks_;
    int obstacle_inflation_radius_;
    double goal_tolerance_;
    std::array<double, 4> rectangular_sampling_limits_;

    bool visualization_;
    visualization_msgs::MarkerArray viz_msg;

    void visualize_sample_nodes(const std::vector<std::array<double, 2>>& sample_nodes)
    {
        viz_msg.markers.clear();
        for (size_t i = 0; i < sample_nodes.size(); i++)
        {
            visualization_msgs::Marker point;
            point.header.frame_id = "/map";
            point.header.stamp = ros::Time::now();
            point.ns = "points";
            point.action = visualization_msgs::Marker::ADD;
            point.pose.orientation.w = 1.0;
            point.id = i;
            point.type = visualization_msgs::Marker::SPHERE;
            point.scale.x = 0.2;
            point.scale.y = 0.2;
            point.scale.z = 0.2;
            point.color.r = 1.0f;
            point.color.g = 0.0f;
            point.color.a = 1.0;
            point.pose.position.x = sample_nodes[i][0];
            point.pose.position.y = sample_nodes[i][1];
            point.lifetime = ros::Duration(10);
            viz_msg.markers.push_back(std::move(point));
        }
        node_pub_.publish(viz_msg);
    }
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