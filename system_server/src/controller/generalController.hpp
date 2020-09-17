#ifndef GENERAL_CONTROLLER_HPP
#define GENERAL_CONTROLLER_HPP

#include "robotController.hpp"
#include "taskController.hpp"
#include "locationController.hpp"
#include <ros/ros.h>
#include <system_server/MsgRequest.h>

class GeneralController
{
private:
    /*ros::NodeHandle nh;
    ros::Subscriber subRequest;
    ros::Subscriber subRobotData;
    std::map<uint32_t, ros::Publisher> pubRobotRequest;
    std::map<uint32_t, ros::Publisher> pubRobotTask;

    void callbackRequest(system_server::MsgRequest &msg);*/


    GeneralDao gdao;
    RobotController rc;
    TaskController tc;
    LocationController lc;

public:
    GeneralController();
    GeneralController(std::string host, std::string user, std::string pass, std::string database);
    ~GeneralController();

    void callScheduler();
    void run();
};

#endif