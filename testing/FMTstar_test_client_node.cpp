#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "fmt_star/plan_srv.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FMTstar_test_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<fmt_star::plan_srv>("FMTstar_search");

    fmt_star::plan_srv srv_message;
    srv_message.request.start_position = std::vector<double>{0, 0};
    srv_message.request.end_position = std::vector<double>{9.5, 7};

    visualization_msgs::MarkerArray viz_msg;

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
