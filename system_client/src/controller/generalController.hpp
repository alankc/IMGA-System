#ifndef GENERAL_CONTROLLER_HPP
#define GENERAL_CONTROLLER_HPP

#include <ros/ros.h>
#include <string>
#include <system_client/MsgRequest.h>
#include <system_client/MsgTaskList.h>
#include <system_client/MsgRobotData.h>
#include <thread>
#include <future>
#include <chrono>

#include "locationController.hpp"
#include "robotController.hpp"
#include "taskController.hpp"
#include "../navigation/navigator.hpp"
#include "../battery/batterySimulator.hpp"

class GeneralController
{
private:
    std::uint32_t robot_id;
    std::string host;
    std::string user;
    std::string pass;
    std::string db;
    std::string navigator_topic;
    std::string navigator_frame;
    std::string server_request;
    std::string server_robot_data;
    double battery_start;
    double battery_noise;

    ros::NodeHandle nh;
    ros::Publisher pubSrvRequest;
    ros::Publisher pubSrvRobotData;
    ros::Subscriber subRequest;
    ros::Subscriber subTask;

    GeneralDao gd;
    LocationController lc;
    RobotController rc;
    TaskController tc;

    Navigator nav;
    volatile bool stopTask;
    volatile bool goToCharge;

    BatterySimulator bs;

    void callbackPubSrvRequest(const system_client::MsgRequest &msg);
    void callbackPubSrvRobotData();
    void callbackRobotCheck();
    void callbackSubRequest(const system_client::MsgRequest &msg);
    void callbackCancelTask(uint32_t id);
    void callbackSubTask(const system_client::MsgTaskList &msg);
    void goToDepot();
    void callbackBattery();

    void performTask(const system_client::MsgTask t);
    void performTasks();

public:
    GeneralController(std::uint32_t robot_id, std::string host, std::string user, std::string pass, std::string db, std::string navigator_topic, std::string navigator_frame, std::string server_request, std::string server_robot_data, double battery_start, double battery_noise);
    ~GeneralController(){};
    void run();
};

#endif