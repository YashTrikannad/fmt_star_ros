#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "fmt_star/plan_srv.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FMTstar_test_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<fmt_star::plan_srv>("FMTstar_search");

    // Get Start and End Co-ordinates
    fmt_star::plan_srv srv_message;
    srv_message.request.start_position = std::vector<double>{0, 0};
    srv_message.request.end_position = std::vector<double>{5, 0};;

    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("sampled_nodes", 1000);

    visualization_msgs::MarkerArray viz_msg;

    while(true)
    {
        // Call the service and get a response
        if (client.call(srv_message))
        {
            viz_msg.markers.clear();
            if (srv_message.response.y_nodes.empty() || srv_message.response.x_nodes.empty())
            {
                ROS_ERROR("Sample Nodes Not Recieved");
            }
            for (size_t i = 0; i < srv_message.response.x_nodes.size(); i++)
            {
                visualization_msgs::Marker point;
                std::cout << "i: " << i << " ";
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
                point.pose.position.x = srv_message.response.x_nodes[i];
                point.pose.position.y = srv_message.response.y_nodes[i];
                point.lifetime = ros::Duration(10);
                viz_msg.markers.push_back(std::move(point));
            }
            pub.publish(viz_msg);
            ROS_INFO("Sample Nodes Published");
            ROS_INFO("Test Nodes Recieved");
        } else
        {
            ROS_ERROR("No Plan Recieved");
        }
    }

    return 0;
}
