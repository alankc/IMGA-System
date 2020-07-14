#include "navigator.hpp"

Navigator::Navigator() {}

Navigator::Navigator(std::string robot, std::string topic, std::string frame)
{
    this->robot = robot;
    this->topic = topic;
    this->frame = frame;
}

Navigator::~Navigator()
{
}

bool Navigator::navigateTo(double x, double y, double w)
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac(robot + "/" + topic, true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = frame;
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = w;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        return true;

    return false;
}