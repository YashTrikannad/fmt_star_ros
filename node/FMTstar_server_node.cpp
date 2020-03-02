#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include "fmt_star/planner.h"
#include "fmt_star/plan_srv.h"


class PlannerService
{
public:
    PlannerService(ros::NodeHandle &nh): nh_(nh)
    {
        bool visualization, online;
        int n_samples, n_collision_checks, obstacle_inflation_radius;
        double near_radius, goal_tolerance, hg_ratio;
        std::array<double, 4> rectangular_sampling_limits{};

        nh_.getParam("n_samples", n_samples);
        nh_.getParam("near_radius", near_radius);
        nh_.getParam("n_collision_checks", n_collision_checks);
        nh_.getParam("obstacle_inflation_radius", obstacle_inflation_radius);
        nh_.getParam("goal_tolerance", goal_tolerance);
        nh_.getParam("x_min", rectangular_sampling_limits[0]);
        nh_.getParam("x_max", rectangular_sampling_limits[1]);
        nh_.getParam("y_min", rectangular_sampling_limits[2]);
        nh_.getParam("y_max", rectangular_sampling_limits[3]);
        nh_.getParam("online", online);
        nh_.getParam("visualization", visualization);
        nh_.getParam("hg_ratio", hg_ratio);

        tree_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("tree_viz", 1000);
        path_pub_ = nh_.advertise<visualization_msgs::Marker>("path_viz", 1000);
        samples_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("samples_viz", 1000);

        input_map_ = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(2));

        planner_ = std::make_unique<fmt_star::Planner>(input_map_,
                                                       n_samples,
                                                       near_radius,
                                                       n_collision_checks,
                                                       obstacle_inflation_radius,
                                                       goal_tolerance,
                                                       hg_ratio,
                                                       online,
                                                       rectangular_sampling_limits,
                                                       visualization,
                                                       &samples_pub_,
                                                       &tree_pub_,
                                                       &path_pub_);
    }

    bool get_plan(fmt_star::plan_srv::Request& request, fmt_star::plan_srv::Response& response)
    {
        ROS_INFO("Start and Goal Recieved by the Planner");
        if(request.update_samples)
        {
            if(request.update_map)
            {
                input_map_ = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(1));
                if(!input_map_)
                {
                    ROS_ERROR("Updated Input Map not Recieved");
                    return false;
                }
            }
            planner_->update_occupancy_grid(input_map_, {request.start_position.pose.position.x,request.start_position.pose.position.y},
                                            {request.end_position.pose.position.x,request.end_position.pose.position.y});
        }

        const auto plan = planner_->get_plan({request.start_position.pose.position.x,request.start_position.pose.position.y},
                                           {request.end_position.pose.position.x,request.end_position.pose.position.y});

        if(plan.empty())
        {
            ROS_INFO("Plan not found");
            return false;
        }

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
        ROS_INFO("Sent Plan");
        return true;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Publisher tree_pub_;
    ros::Publisher samples_pub_;

    std::unique_ptr<fmt_star::Planner> planner_;
    nav_msgs::OccupancyGridConstPtr input_map_;
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