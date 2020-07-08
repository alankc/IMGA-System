#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <string>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Navigation
{
private:
    std::string robot;
    std::string topic;
    std::string frame;
public:
    Navigation(std::string robot, std::string topic, std::string frame);
    ~Navigation();
    bool navigateTo(double x, double y, double w);
};

#endif