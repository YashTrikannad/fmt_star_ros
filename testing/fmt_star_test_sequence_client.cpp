#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <fmt_star/planAction.h>
#include <nav_msgs/OccupancyGrid.h>

std::vector<std::array<double, 2>> translate_sequence_to_ros_coords(const std::vector<std::array<int, 2>>& nonros_coords,
                                                                    const double nonros_map_width,
                                                                    const double nonros_map_height,
                                                                    const double resolution,
                                                                    const bool xy_switched,
                                                                    const double ros_map_width,
                                                                    const double ros_map_height,
                                                                    const double ros_map_origin_x,
                                                                    const double ros_map_origin_y)
{
    std::vector<std::array<double, 2>> ros_coords;
    int x_index = xy_switched ? 1 : 0;
    int y_index = xy_switched ? 0 : 1;
    for(const auto& nonros_coord: nonros_coords)
    {
        const double x = ros_map_origin_x + (nonros_coord[x_index] * ros_map_width * resolution) / (nonros_map_width);
        const double y =
                ros_map_origin_y + ros_map_height * resolution * (1 - (nonros_coord[y_index] / nonros_map_height));
        ros_coords.emplace_back(std::array<double, 2>{x, y});
    }
    return ros_coords;
}

int main (int argc, char **argv)
{
    // Initialize ROS Node
    ros::init(argc, argv, "fmt_star_test_sequence_client");
    ros::NodeHandle nh;

    // Test Sequence
    std::vector<std::array<int, 2>> sequence{std::array<int, 2>{400, 973},
                                             std::array<int, 2>{413, 793},
                                             std::array<int, 2>{461, 500},
                                             std::array<int, 2>{114, 112}};
    const double original_map_height = 1134;
    const double original_map_width = 1355;

    ROS_INFO("Waiting for Map.");
    const auto input_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(2));
    if(input_map == nullptr) return -1;
    ROS_INFO("Map Receieved.");
    const double resolution = input_map->info.resolution;
    const auto origin = input_map->info.origin;
    ROS_INFO("Resolution: %f", resolution);
    ROS_INFO("Origin: (%f, %f)", origin.position.x, origin.position.y);
    ROS_INFO("Map Size: (%f, %f)", input_map->info.width*resolution, input_map->info.height*resolution);

    const auto ros_sequence = translate_sequence_to_ros_coords(sequence,
                                                             original_map_width,
                                                             original_map_height,
                                                             resolution,
                                                             true,
                                                             input_map->info.width*resolution,
                                                             input_map->info.height*resolution,
                                                             origin.position.x,
                                                             origin.position.y);

    actionlib::SimpleActionClient<fmt_star::planAction> ac("fmt_star_server", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending seqence one by one.");

    for(int i=0; i<sequence.size()-1; i++)
    {
        geometry_msgs::PoseStamped start{};

        start.pose.position.x = ros_sequence[i][0];
        start.pose.position.y = ros_sequence[i][1];
        start.pose.orientation.z = 1;
        ROS_INFO("Start Position: (%f, %f)", start.pose.position.x, start.pose.position.y);

        geometry_msgs::PoseStamped end;
        end.pose.position.x = ros_sequence[i+1][0];
        end.pose.position.y = ros_sequence[i+1][1];
        end.pose.orientation.z = 1;
        ROS_INFO("End Position: (%f, %f)", end.pose.position.x, end.pose.position.y);

        fmt_star::planGoal goal;
        goal.start_position = start;
        goal.end_position = end;
        goal.update_map = false;
        goal.update_samples = true;
        ac.sendGoal(goal);

        sleep(10);

        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished. Plan Recieved: %s",state.toString().c_str());
        }
        else
        {
            ROS_INFO("Action did not finish before the time out.");
            return -1;
        }
    }
    return 0;
}

