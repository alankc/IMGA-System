#ifndef GENERAL_CONTROLLER_HPP
#define GENERAL_CONTROLLER_HPP

#include <ros/ros.h>
#include <string>
#include <system_client/MsgRequest.h>
#include <system_client/MsgTaskList.h>
#include <system_client/MsgRobotData.h>
#include <mutex>
#include <thread>
#include <future>
#include <chrono>

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
    //std::future navFtr;
    std::mutex navMtx;
    std::unique_lock<std::mutex> navLock;

    void callbackPubSrvRequest(const system_client::MsgRequest &msg);
    void callbackPubSrvRobotData();
    void callbackSubRequest(const system_client::MsgRequest &msg);
    void callbackSubTask(const system_client::MsgTaskList &msg);

    void performTask(const system_client::MsgTask &t);
    void performTasks();
public:
    GeneralController(){};
    ~GeneralController(){};
    void run();
};

#endif