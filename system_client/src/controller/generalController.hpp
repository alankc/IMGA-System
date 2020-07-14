#ifndef GENERAL_CONTROLLER_HPP
#define GENERAL_CONTROLLER_HPP

#include <ros/ros.h>
#include <string>
#include <system_client/MsgRequest.h>
#include <system_client/MsgTaskList.h>
#include <system_client/MsgRobotData.h>
#include <mutex>

#include "locationController.hpp"
#include "robotController.hpp"
#include "taskController.hpp"
#include "../navigation/navigator.hpp"

class GeneralController
{
private:
    std::string srvRequestTopic;
    std::string srvRobotDataTopic;
    std::string myResquestTopic;
    std::string myTaskTopic;

    ros::NodeHandle nh;
    //ros::Publisher pubSrvRequest;
    //ros::Publisher pubSrvRobotData;
    ros::Subscriber subRequest;
    ros::Subscriber subTask;

    GeneralDao gd;
    LocationController lc;
    RobotController rc;
    TaskController tc;

    Navigator nav;

    void callbackPubSrvRequest(const system_client::MsgRequest &msg);
    void callbackPubSrvRobotData();
    void callbackSubRequest(const system_client::MsgRequest &msg);
    void callbackSubTask(const system_client::MsgTaskList &msg);

    void navigation();
public:
    GeneralController(){};
    ~GeneralController(){};
    void run();
};

#endif