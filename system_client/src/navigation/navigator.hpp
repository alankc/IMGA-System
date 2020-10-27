#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP

#include <string>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Navigator
{
private:
    std::string robot;
    std::string topic;
    std::string frame;
    std::unique_ptr<MoveBaseClient> mbc;

public:
    Navigator();
    Navigator(std::string topic, std::string frame);
    ~Navigator();
    void start();
    void navigateTo(double x, double y, double w);
    void cancel();
    bool stillNavigating();
    bool hasArrived();

    Navigator& operator= (const Navigator &navigator);
};

#endif