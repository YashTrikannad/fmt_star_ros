#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "fmt_star/plan_srv.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FMTstar_test_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<fmt_star::plan_srv>("FMTstar_search");

    fmt_star::plan_srv srv_message;

    const auto start = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("gt_pose",ros::Duration(2));
    ROS_INFO("Send a 2d Nav Goal");
    const auto goal = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("move_base_simple/goal",ros::Duration(20));

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
        srv_message.request.update_map = false;
        srv_message.request.update_samples = true;
    }

    if (client.call(srv_message))
    {
        ROS_INFO("Plan Recieved");
    }
    else
    {
        ROS_ERROR("No Plan Recieved");
    }

    return 0;
}
