#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <fmt_star/planAction.h>

#include "fmt_star/planner.h"

class PlannerAction
{
public:
    PlannerAction():
            as_(nh_, "fmt_star_server", boost::bind(&PlannerAction::execute_callback, this, _1), false)
    {
        bool visualization, online, minimal_sampling;
        int n_samples, n_collision_checks, obstacle_inflation_radius;
        double near_radius, goal_tolerance, hg_ratio, sampling_tolerance;
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
        nh_.getParam("minimal_sampling", minimal_sampling);
        nh_.getParam("online", online);
        nh_.getParam("visualization", visualization);
        nh_.getParam("hg_ratio", hg_ratio);
        nh_.getParam("sampling_tolerance", sampling_tolerance);

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
                                                       minimal_sampling,
                                                       sampling_tolerance,
                                                       visualization,
                                                       &samples_pub_,
                                                       &tree_pub_,
                                                       &path_pub_);

        as_.start();
        ROS_INFO("Global Planner Action Ready.");
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<fmt_star::planAction> as_;
    fmt_star::planResult result_;

    ros::Publisher path_pub_;
    ros::Publisher tree_pub_;
    ros::Publisher samples_pub_;

    std::unique_ptr<fmt_star::Planner> planner_;
    nav_msgs::OccupancyGridConstPtr input_map_;

    void execute_callback(const fmt_star::planGoalConstPtr &goal)
    {
        ros::Rate r(1);
        bool success = true;

        ROS_INFO("Start and Goal Recieved by the Planner");
        if(goal->update_samples)
        {
            if(goal->update_map)
            {
                input_map_ = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(1));
                if(!input_map_)
                {
                    ROS_ERROR("Updated Input Map not Recieved");
                    success = false;
                }
            }
            if(success)
            {
                planner_->update_occupancy_grid(input_map_,
                        {goal->start_position.pose.position.x,goal->start_position.pose.position.y},
                        {goal->end_position.pose.position.x,goal->end_position.pose.position.y});
            }
        }

        const auto plan = planner_->get_plan({goal->start_position.pose.position.x,goal->start_position.pose.position.y},
                                             {goal->end_position.pose.position.x,goal->end_position.pose.position.y});

        if(!success || plan.empty())
        {
            ROS_ERROR("Not able to find a path.");
            as_.setAborted(result_);
        }
        else
        {
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
            result_.path = path;
            ROS_INFO("Succeeded");
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fmt_star_server");
    PlannerAction planner;
    ros::spin();
    return 0;
}