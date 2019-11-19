#include <ros/ros.h>
#include "fmt_star/plan_srv.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FMTstar_test_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<fmt_star::plan_srv>("FMTstar_search");

    // Get Start and End Co-ordinates
    fmt_star::plan_srv srv_message;
    srv_message.request.start_index = 1;
    srv_message.request.end_index = 100;

    // Call the service and get a response
    if(client.call(srv_message))
    {
        ROS_INFO("Plan Recieved");
    }
    else
    {
        ROS_ERROR("No Plan Recieved");
    }

    return 0;
}
