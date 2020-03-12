#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <fmt_star/planAction.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "fmt_star_test_client");

    actionlib::SimpleActionClient<fmt_star::planAction> ac("fmt_star_server", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    const auto start = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("gt_pose_1",ros::Duration(2));
    ROS_INFO("Send a 2d Nav Goal");
    const auto end = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("move_base_simple/goal",ros::Duration(20));

    fmt_star::planGoal goal;

    if(!start)
    {
        ROS_ERROR("Unable to Plan. Start not recieved");
    }
    else if(!end)
    {
        ROS_ERROR("Unable to Plan. Goal not recieved");
    }
    else
    {
        goal.start_position = *start;
        goal.end_position = *end;
        goal.update_map = false;
        goal.update_samples = true;
    }

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished. Plan Recieved: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    return 0;
}
