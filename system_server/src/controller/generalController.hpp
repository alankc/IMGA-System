#ifndef GENERAL_CONTROLLER_HPP
#define GENERAL_CONTROLLER_HPP

#include "robotController.hpp"
#include "taskController.hpp"
#include "locationController.hpp"
#include "../dao/settingsDao.hpp"
#include <ros/ros.h>
#include <system_server/MsgRequest.h>

class GeneralController
{
private:
    ros::NodeHandle nh;
    ros::Subscriber subRequest;
    ros::Subscriber subRobotData;

    void callbackRequest(const system_server::MsgRequest &msg);
    void callbackRobotData(const system_server::MsgRobotData &msg);
    void callbackRequestRobotCheck(const system_server::MsgRequest &msg);
    void callbackRequestChargeBattery(const system_server::MsgRequest &msg);
    void callbackRequestCancelTask(const system_server::MsgRequest &msg);
    void callbackRequestPerformingPickUp(const system_server::MsgRequest &msg);
    void callbackRequestPerformingDelivery(const system_server::MsgRequest &msg);
    void callbackRequestSucessTask(const system_server::MsgRequest &msg);
    void callbackRequestFailTask(const system_server::MsgRequest &msg);

    GeneralDao gdao;
    SettingsDao sDao;
    Settings settings;
    
    RobotController rc;
    TaskController tc;
    LocationController lc;

    //Used to get current time in seconds
    std::time_t tt;
    std::tm bt;
    std::chrono::_V2::system_clock::time_point zero;

    double getCurrentTime_ms();
    double getCurrentTime_s();
    void callScheduler();
    void schedulingLoop();

public:
    GeneralController();
    GeneralController(std::string host, std::string user, std::string pass, std::string database);
    ~GeneralController();

    void run();
};

#endif