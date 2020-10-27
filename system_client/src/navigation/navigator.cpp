#include "navigator.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <tf/tf.h>

Navigator::Navigator()
{
    this->topic = "move_base";
    this->frame = "map";
}

Navigator::Navigator(std::string topic, std::string frame)
{
    this->topic = topic;
    this->frame = frame;
}

Navigator::~Navigator()
{
}

void Navigator::start()
{
    mbc.reset(new MoveBaseClient(topic, true));
    while (!mbc->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
}

void Navigator::navigateTo(double x, double y, double w)
{
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = frame;
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    tf::Quaternion qt;
    qt.setRPY(0, 0, w);

    goal.target_pose.pose.orientation.x = qt.getX();
    goal.target_pose.pose.orientation.y = qt.getY();
    goal.target_pose.pose.orientation.z = qt.getZ();
    goal.target_pose.pose.orientation.w = qt.getW();

    mbc->sendGoal(goal);
}

void Navigator::cancel()
{
    mbc->cancelAllGoals();
}

bool Navigator::stillNavigating()
{
    auto stt = mbc->getState();
    return (stt == actionlib::SimpleClientGoalState::ACTIVE) || (stt == actionlib::SimpleClientGoalState::PENDING);
}

bool Navigator::hasArrived()
{
    auto stt = mbc->getState();
    return (stt == actionlib::SimpleClientGoalState::SUCCEEDED);
}

Navigator &Navigator::operator=(const Navigator &navigator)
{
    if (this != &navigator)
    {
        topic = navigator.topic;
        frame = navigator.frame;
        mbc.reset(navigator.mbc.get());
    }
}